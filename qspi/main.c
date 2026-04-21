// SPDX-License-Identifier: MIT
// main.c --- SPI2 PRBS throughput and integrity demo
// Copyright (c) 2026 Jakob Kastelic

// Stress-tests the SPI2 peripheral as a high-rate link.
// Communication over UART0 is control-plane only; all bulk
// data flows over SPI2 while the DSP either verifies the
// stream against a PRBS reference, generates one, or
// discards bytes. Runs in either slave or master role.
//
// UART command protocol
// ---------------------
// Each command is an ASCII line terminated by '\n' or '\r'.
// Whitespace-separated tokens. Numbers are decimal unless
// prefixed 0x. Supported commands:
//
//   Rs
//      Slave role (default). DSP receives clock and CS from
//      the external master.
//
//   Rm <clkdiv>
//      Master role with SPI_CLK divisor <clkdiv>; bit rate is
//      SCLK0 / (clkdiv + 1). DSP drives CLK and SEL1.
//
//   M1 | M2 | M4
//      Reconfigure the SPI port for 1-, 2-, or 4-lane I/O.
//      Takes effect immediately; issue while the bus is idle.
//
//   P <seed_hex> <count>
//      PRBS op. In slave role: drain `count` bytes from SPI
//      and compare each against xorshift32(<seed>). In master
//      role: generate the same PRBS stream and clock it out.
//      Prints one summary line.
//
//   V <seed_hex> <count>
//      Master-only. Clock `count` bytes with MOSI driven to 0
//      and verify incoming MISO bytes against xorshift32(seed).
//
//   X <seed_hex> <count>
//      Master-only. Full-duplex PRBS transfer: MOSI drives
//      xorshift32(seed), MISO verified against the same seed.
//
//   L <count>
//      Raw sink / source. Slave role drains and discards;
//      master role clocks out `count` zero bytes. Useful for
//      baseline throughput and for provoking ROR under load.
//
//   D <seed_hex> <count>
//      Slave-only PRBS verify routed through the SPI2_RX
//      peripheral DMA channel (DMA27). Receives in-SRAM in
//      16 KiB chunks, waits for the DMA to finish, then
//      walks the buffer checking against the PRBS reference.
//
//   ?
//      Print firmware state (role, lane mode).
//
// After every data op the firmware prints
//   PRBS role=.. mode=.. seed=0x.. N=.. OK ticks=.. ror=..
// or the FAIL variant, then re-enters the command loop.
//
// PRBS algorithm (bit-exact with
// test_serv/examples/make_qspi.py :: prbs_xorshift32):
//
//   x ^= x << 13; x ^= x >> 17; x ^= x << 5; emit x & 0xFF;
//
// Pin mapping (PA0..PA5, alternate function "b"):
//   PA0 = SPI2_MISO / D1
//   PA1 = SPI2_MOSI / D0
//   PA2 = SPI2_D2
//   PA3 = SPI2_D3
//   PA4 = SPI2_CLK
//   PA5 = SPI2_SEL1
//
// SPI is always configured CPOL=0, CPHA=0, MSB-first, 32-bit
// word size. The inner loop drains one FIFO slot per four
// bytes. Master-role throughput is limited by SCLK0 / (clkdiv
// + 1). DMA support is slave-RX only in this iteration; a
// matching master-TX DMA path can be added later.

#include "board.h"
#include "clocks.h"
#include "dma.h"
#include "pinmux.h"
#include "regs.h"
#include "spi.h"
#include "timer.h"
#include "uart.h"
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#define SPI_PORT       SPI_ID_2
#define SPI_RX_DMA     DMA_CH_SPI2_RX
#define SPI_STRIDE     0x1000U
#define STARTUP_MS     500U
#define BYTES_PER_WORD 4U
#define BITS_PER_BYTE  8U
#define BYTE_MASK      0xFFU
#define WORD_MSB_SHIFT 24U // (BYTES_PER_WORD - 1) * BITS_PER_BYTE
#define CMD_BUF_SIZE   128U
#define BASE_DEC       10U
#define BASE_HEX       16U
// xorshift32 feedback triple tuned by Marsaglia (Journal of
// Statistical Software, 8(14)). Period 2^32-1.
#define PRBS_SHIFT_A 13U
#define PRBS_SHIFT_B 17U
#define PRBS_SHIFT_C 5U

// DMA receive buffer size, in 32-bit words. Ops with larger
// count re-arm the DMA channel in chunks of this size.
#define DMA_BUF_WORDS (16U * 1024U / 4U) // 16 KiB

static uint32_t dma_rx_buf[DMA_BUF_WORDS];

static uint32_t spi_base;
static enum spi_miom current_miom = SPI_MIO_SINGLE;
static bool current_master        = false;
static uint32_t current_clkdiv    = 0;

// --------- PRBS ---------

static inline uint8_t prbs_next(uint32_t *state)
{
   uint32_t x = *state;
   x ^= x << PRBS_SHIFT_A;
   x ^= x >> PRBS_SHIFT_B;
   x ^= x << PRBS_SHIFT_C;
   *state = x;
   return (uint8_t)(x & BYTE_MASK);
}

// --------- SPI helpers ---------

static void spi_reconfigure(enum spi_miom miom, bool master, uint32_t clkdiv)
{
   struct spi_cfg cfg = {
       .clkdiv    = clkdiv,
       .size      = SPI_WORD_32,
       .miom      = miom,
       .is_master = master ? 1U : 0U,
       .cpol      = 0,
       .cpha      = 0,
       .lsb_first = 0,
   };
   spi_init(SPI_PORT, &cfg);
   if (!master)
      spi_rx_dma_enable(SPI_PORT);
   spi_tx_enable(SPI_PORT);
   spi_rx_enable(SPI_PORT);
   current_miom   = miom;
   current_master = master;
   current_clkdiv = clkdiv;
}

static void spi_rx_flush(void)
{
   while (!(MMR(spi_base + OFF_SPI_STAT) & BIT_SPI_STAT_RFE))
      (void)MMR(spi_base + OFF_SPI_RFIFO);
   // W1C all latched error/complete bits.
   MMR(spi_base + OFF_SPI_STAT) =
       BIT_SPI_STAT_ROR | BIT_SPI_STAT_TUR | BIT_SPI_STAT_TC;
}

// --------- UART line reader ---------

// Blocking: return the next complete command line. Lines are
// terminated by '\n' or '\r'; both CR and LF are treated as
// separators. Empty lines (e.g. "\r\n" pairs) are skipped.
// Bytes overflowing CMD_BUF_SIZE are dropped from the tail;
// the buffer is always NUL-terminated.
static void uart_read_line(char *buf, uint32_t cap)
{
   uint32_t n = 0;
   for (;;) {
      int c = uart_try_getc();
      if (c < 0)
         continue;
      if (c == '\r' || c == '\n') {
         if (n == 0)
            continue;
         break;
      }
      if (n + 1U < cap)
         buf[n++] = (char)c;
   }
   buf[n] = '\0';
}

// --------- Tiny parser helpers ---------

static bool is_space(char c)
{
   return c == ' ' || c == '\t';
}

// Skip leading whitespace, return pointer to first non-space.
static const char *skip_ws(const char *s)
{
   while (is_space(*s))
      s++;
   return s;
}

// Parse one unsigned integer with an explicit default base.
// If the input begins with "0x" the base is forced to 16; else
// the default_base argument is used. Writes the value to *out
// and returns the first unconsumed character. On no digits,
// returns the input unchanged and leaves *out unmodified.
static const char *parse_u32_base(const char *s, uint32_t *out,
                                  unsigned default_base)
{
   s             = skip_ws(s);
   uint32_t v    = 0;
   bool got      = false;
   unsigned base = default_base;
   if (s[0] == '0' && (s[1] == 'x' || s[1] == 'X')) {
      base = BASE_HEX;
      s += 2;
   }
   for (;;) {
      char c     = *s;
      unsigned d = 0;
      if (c >= '0' && c <= '9')
         d = (unsigned)(c - '0');
      else if (base == BASE_HEX && c >= 'a' && c <= 'f')
         d = BASE_DEC + (unsigned)(c - 'a');
      else if (base == BASE_HEX && c >= 'A' && c <= 'F')
         d = BASE_DEC + (unsigned)(c - 'A');
      else
         break;
      v   = v * base + d;
      got = true;
      s++;
   }
   if (got)
      *out = v;
   return s;
}

// Convenience wrappers.
static const char *parse_u32(const char *s, uint32_t *out)
{
   return parse_u32_base(s, out, BASE_DEC);
}

static const char *parse_u32_hex(const char *s, uint32_t *out)
{
   return parse_u32_base(s, out, BASE_HEX);
}

// --------- Op implementations ---------

static const char *miom_name(enum spi_miom m)
{
   switch (m) {
      case SPI_MIO_SINGLE: return "x1";
      case SPI_MIO_DUAL: return "x2";
      case SPI_MIO_QUAD: return "x4";
   }
   return "??";
}

static void op_prbs(uint32_t seed, uint32_t count)
{
   spi_rx_flush();

   uint32_t state      = seed ? seed : 1U;
   uint32_t mismatches = 0;
   uint32_t first_at   = 0;
   uint32_t idx        = 0;
   uint32_t words      = count / BYTES_PER_WORD;

   uint32_t t0 = timer_ticks();
   for (uint32_t i = 0; i < words; i++) {
      while (MMR(spi_base + OFF_SPI_STAT) & BIT_SPI_STAT_RFE)
         ;
      uint32_t w = MMR(spi_base + OFF_SPI_RFIFO);
      // MSB-first: first byte received is in bits [31:24].
      for (unsigned b = 0; b < BYTES_PER_WORD; b++) {
         unsigned shift = WORD_MSB_SHIFT - (b * BITS_PER_BYTE);
         uint8_t got    = (uint8_t)(w >> shift);
         uint8_t exp    = prbs_next(&state);
         if (got != exp) {
            if (mismatches == 0)
               first_at = idx;
            mismatches++;
         }
         idx++;
      }
   }
   uint32_t elapsed = timer_ticks() - t0;

   uint32_t stat = MMR(spi_base + OFF_SPI_STAT);
   unsigned ror  = (stat & BIT_SPI_STAT_ROR) ? 1U : 0U;
   MMR(spi_base + OFF_SPI_STAT) =
       BIT_SPI_STAT_ROR | BIT_SPI_STAT_TUR | BIT_SPI_STAT_TC;

   if (mismatches == 0 && !ror) {
      printf("PRBS mode=%s seed=0x%08x N=%u OK ticks=%u ror=%u\r\n",
             miom_name(current_miom), seed, count, elapsed, ror);
   } else {
      printf("PRBS mode=%s seed=0x%08x N=%u FAIL mismatches=%u first_at=%u "
             "ticks=%u ror=%u\r\n",
             miom_name(current_miom), seed, count, mismatches, first_at,
             elapsed, ror);
   }
}

// PRBS verify using the SPI2_RX peripheral DMA. The transfer
// is broken into chunks of DMA_BUF_WORDS * 4 bytes; each chunk
// is DMAed into dma_rx_buf then walked byte-by-byte against
// the expected PRBS stream before the next chunk is armed.
static void op_prbs_dma(uint32_t seed, uint32_t count)
{
   spi_rx_flush();

   uint32_t state      = seed ? seed : 1U;
   uint32_t mismatches = 0;
   uint32_t first_at   = 0;
   uint32_t idx        = 0;
   uint32_t remaining  = count / BYTES_PER_WORD;

   uint32_t t0 = timer_ticks();
   while (remaining > 0U) {
      uint32_t chunk_words =
          remaining < DMA_BUF_WORDS ? remaining : DMA_BUF_WORDS;

      dma_oneshot_config(SPI_RX_DMA, (struct dma_buf){dma_rx_buf, chunk_words},
                         DMA_DIR_RX_TO_MEM);
      dma_enable(SPI_RX_DMA);

      while (!dma_done(SPI_RX_DMA))
         ;
      dma_disable(SPI_RX_DMA);

      for (uint32_t i = 0; i < chunk_words; i++) {
         uint32_t w = dma_rx_buf[i];
         for (unsigned b = 0; b < BYTES_PER_WORD; b++) {
            unsigned shift = WORD_MSB_SHIFT - (b * BITS_PER_BYTE);
            uint8_t got    = (uint8_t)(w >> shift);
            uint8_t exp    = prbs_next(&state);
            if (got != exp) {
               if (mismatches == 0)
                  first_at = idx;
               mismatches++;
            }
            idx++;
         }
      }
      remaining -= chunk_words;
   }
   uint32_t elapsed = timer_ticks() - t0;

   uint32_t stat = MMR(spi_base + OFF_SPI_STAT);
   unsigned ror  = (stat & BIT_SPI_STAT_ROR) ? 1U : 0U;
   MMR(spi_base + OFF_SPI_STAT) =
       BIT_SPI_STAT_ROR | BIT_SPI_STAT_TUR | BIT_SPI_STAT_TC;

   if (mismatches == 0 && !ror) {
      printf("PRBSDMA mode=%s seed=0x%08x N=%u OK ticks=%u ror=%u\r\n",
             miom_name(current_miom), seed, count, elapsed, ror);
   } else {
      printf("PRBSDMA mode=%s seed=0x%08x N=%u FAIL mismatches=%u first_at=%u "
             "ticks=%u ror=%u\r\n",
             miom_name(current_miom), seed, count, mismatches, first_at,
             elapsed, ror);
   }
}

// Master-role PRBS transmit. Generates the same xorshift32
// stream as the slave verifier and clocks it out on MOSI (or
// on the active data lanes in dual/quad mode). In full-duplex
// single-lane mode the slave's MISO output is ignored.
static void op_prbs_tx(uint32_t seed, uint32_t count)
{
   // Drain any stale RX before starting (full-duplex will
   // refill it as clocks run).
   spi_rx_flush();

   uint32_t state = seed ? seed : 1U;
   uint32_t words = count / BYTES_PER_WORD;

   uint32_t t0 = timer_ticks();
   for (uint32_t i = 0; i < words; i++) {
      uint32_t w = 0;
      for (unsigned b = 0; b < BYTES_PER_WORD; b++) {
         unsigned shift = WORD_MSB_SHIFT - (b * BITS_PER_BYTE);
         w |= (uint32_t)prbs_next(&state) << shift;
      }
      while (MMR(spi_base + OFF_SPI_STAT) & BIT_SPI_STAT_TFF)
         ;
      MMR(spi_base + OFF_SPI_TFIFO) = w;
   }
   // Wait for the TX shift register and FIFO to fully drain.
   while (MMR(spi_base + OFF_SPI_STAT) & BIT_SPI_STAT_TS)
      ;
   uint32_t elapsed = timer_ticks() - t0;

   uint32_t stat = MMR(spi_base + OFF_SPI_STAT);
   unsigned tur  = (stat & BIT_SPI_STAT_TUR) ? 1U : 0U;
   MMR(spi_base + OFF_SPI_STAT) =
       BIT_SPI_STAT_ROR | BIT_SPI_STAT_TUR | BIT_SPI_STAT_TC;

   printf("PRBSTX mode=%s seed=0x%08x N=%u ticks=%u tur=%u\r\n",
          miom_name(current_miom), seed, count, elapsed, tur);
}

// Master-role zero-byte push. Clocks out `count` zero bytes
// at the configured rate. Slave-role loopback is handled by
// op_loopback below.
static void op_tx_zeros(uint32_t count)
{
   uint32_t words = count / BYTES_PER_WORD;
   uint32_t t0    = timer_ticks();
   for (uint32_t i = 0; i < words; i++) {
      while (MMR(spi_base + OFF_SPI_STAT) & BIT_SPI_STAT_TFF)
         ;
      MMR(spi_base + OFF_SPI_TFIFO) = 0U;
   }
   while (MMR(spi_base + OFF_SPI_STAT) & BIT_SPI_STAT_TS)
      ;
   uint32_t elapsed = timer_ticks() - t0;

   uint32_t stat = MMR(spi_base + OFF_SPI_STAT);
   unsigned tur  = (stat & BIT_SPI_STAT_TUR) ? 1U : 0U;
   MMR(spi_base + OFF_SPI_STAT) =
       BIT_SPI_STAT_ROR | BIT_SPI_STAT_TUR | BIT_SPI_STAT_TC;

   printf("TX0 mode=%s N=%u ticks=%u tur=%u\r\n", miom_name(current_miom),
          count, elapsed, tur);
}

// Full-duplex helper. Does one iteration of the master
// push/pop inner loop: push tx_word into TFIFO, then wait for
// the corresponding RX word and verify each byte against the
// PRBS reference. *mismatches / *first_at / *idx are updated
// in place.
static inline void fd_step(uint32_t tx_word, uint32_t *state,
                           uint32_t *mismatches, uint32_t *first_at,
                           uint32_t *idx)
{
   while (MMR(spi_base + OFF_SPI_STAT) & BIT_SPI_STAT_TFF)
      ;
   MMR(spi_base + OFF_SPI_TFIFO) = tx_word;
   while (MMR(spi_base + OFF_SPI_STAT) & BIT_SPI_STAT_RFE)
      ;
   uint32_t w = MMR(spi_base + OFF_SPI_RFIFO);
   for (unsigned b = 0; b < BYTES_PER_WORD; b++) {
      unsigned shift = WORD_MSB_SHIFT - (b * BITS_PER_BYTE);
      uint8_t got    = (uint8_t)(w >> shift);
      uint8_t exp    = prbs_next(state);
      if (got != exp) {
         if (*mismatches == 0U)
            *first_at = *idx;
         (*mismatches)++;
      }
      (*idx)++;
   }
}

// Master-role PRBS receive+verify. Clocks `count` bytes with
// TX driven to 0 (MOSI don't-care) while the slave drives
// PRBS on MISO; each RX word is verified byte-by-byte.
static void op_prbs_rx_master(uint32_t seed, uint32_t count)
{
   spi_rx_flush();

   uint32_t state      = seed ? seed : 1U;
   uint32_t mismatches = 0;
   uint32_t first_at   = 0;
   uint32_t idx        = 0;
   uint32_t words      = count / BYTES_PER_WORD;

   uint32_t t0 = timer_ticks();
   for (uint32_t i = 0; i < words; i++)
      fd_step(0U, &state, &mismatches, &first_at, &idx);
   uint32_t elapsed = timer_ticks() - t0;

   uint32_t stat = MMR(spi_base + OFF_SPI_STAT);
   unsigned tur  = (stat & BIT_SPI_STAT_TUR) ? 1U : 0U;
   MMR(spi_base + OFF_SPI_STAT) =
       BIT_SPI_STAT_ROR | BIT_SPI_STAT_TUR | BIT_SPI_STAT_TC;

   if (mismatches == 0U) {
      printf("PRBSRX mode=%s seed=0x%08x N=%u OK ticks=%u tur=%u\r\n",
             miom_name(current_miom), seed, count, elapsed, tur);
   } else {
      printf("PRBSRX mode=%s seed=0x%08x N=%u FAIL mismatches=%u first_at=%u "
             "ticks=%u tur=%u\r\n",
             miom_name(current_miom), seed, count, mismatches, first_at,
             elapsed, tur);
   }
}

// Master-role full-duplex PRBS transfer. TX drives PRBS out
// on MOSI and RX verifies PRBS coming in on MISO; both sides
// use the same seed, so the slave must generate the identical
// stream to pass.
static void op_prbs_xfer(uint32_t seed, uint32_t count)
{
   spi_rx_flush();

   uint32_t tx_state   = seed ? seed : 1U;
   uint32_t rx_state   = tx_state;
   uint32_t mismatches = 0;
   uint32_t first_at   = 0;
   uint32_t idx        = 0;
   uint32_t words      = count / BYTES_PER_WORD;

   uint32_t t0 = timer_ticks();
   for (uint32_t i = 0; i < words; i++) {
      uint32_t w = 0;
      for (unsigned b = 0; b < BYTES_PER_WORD; b++) {
         unsigned shift = WORD_MSB_SHIFT - (b * BITS_PER_BYTE);
         w |= (uint32_t)prbs_next(&tx_state) << shift;
      }
      fd_step(w, &rx_state, &mismatches, &first_at, &idx);
   }
   uint32_t elapsed = timer_ticks() - t0;

   uint32_t stat = MMR(spi_base + OFF_SPI_STAT);
   unsigned tur  = (stat & BIT_SPI_STAT_TUR) ? 1U : 0U;
   MMR(spi_base + OFF_SPI_STAT) =
       BIT_SPI_STAT_ROR | BIT_SPI_STAT_TUR | BIT_SPI_STAT_TC;

   if (mismatches == 0U) {
      printf("PRBSXF mode=%s seed=0x%08x N=%u OK ticks=%u tur=%u\r\n",
             miom_name(current_miom), seed, count, elapsed, tur);
   } else {
      printf("PRBSXF mode=%s seed=0x%08x N=%u FAIL mismatches=%u first_at=%u "
             "ticks=%u tur=%u\r\n",
             miom_name(current_miom), seed, count, mismatches, first_at,
             elapsed, tur);
   }
}

static void op_loopback(uint32_t count)
{
   spi_rx_flush();

   uint32_t words = count / BYTES_PER_WORD;
   uint32_t t0    = timer_ticks();
   uint32_t acc   = 0;
   for (uint32_t i = 0; i < words; i++) {
      while (MMR(spi_base + OFF_SPI_STAT) & BIT_SPI_STAT_RFE)
         ;
      acc ^= MMR(spi_base + OFF_SPI_RFIFO);
   }
   uint32_t elapsed = timer_ticks() - t0;

   uint32_t stat = MMR(spi_base + OFF_SPI_STAT);
   unsigned ror  = (stat & BIT_SPI_STAT_ROR) ? 1U : 0U;
   MMR(spi_base + OFF_SPI_STAT) =
       BIT_SPI_STAT_ROR | BIT_SPI_STAT_TUR | BIT_SPI_STAT_TC;

   printf("LOOP mode=%s N=%u xor=%08x ticks=%u ror=%u\r\n",
          miom_name(current_miom), count, acc, elapsed, ror);
}

// --------- Command dispatch ---------

// Parse "<seed_hex> <count>" from rest. Returns true on valid
// parse (count > 0 and multiple of 4), false and prints an
// error otherwise.
static bool parse_seed_count(const char *rest, uint32_t *seed, uint32_t *count)
{
   // Seed is documented as hex ("<seed_hex>"), so default to base 16
   // to let bare "c0ffee" parse without a 0x prefix. Count stays
   // decimal by default.
   const char *p = parse_u32_hex(rest, seed);
   (void)parse_u32(p, count);
   if (*count == 0U || (*count & 3U) != 0U) {
      printf("ERR count must be >0 and a multiple of 4\r\n");
      return false;
   }
   return true;
}

static void cmd_role(const char *rest)
{
   if (rest[0] == 's') {
      spi_reconfigure(current_miom, false, 0);
      printf("ROLE slave mode=%s\r\n", miom_name(current_miom));
   } else if (rest[0] == 'm') {
      uint32_t clkdiv = 0;
      (void)parse_u32(rest + 1, &clkdiv);
      spi_reconfigure(current_miom, true, clkdiv);
      printf("ROLE master clkdiv=%u mode=%s\r\n", clkdiv,
             miom_name(current_miom));
   } else {
      printf("ERR bad role\r\n");
   }
}

static void cmd_mode(const char *rest)
{
   enum spi_miom m = SPI_MIO_SINGLE;
   switch (rest[0]) {
      case '1': m = SPI_MIO_SINGLE; break;
      case '2': m = SPI_MIO_DUAL; break;
      case '4': m = SPI_MIO_QUAD; break;
      default: printf("ERR bad mode\r\n"); return;
   }
   spi_reconfigure(m, current_master, current_clkdiv);
   printf("MODE %s\r\n", miom_name(current_miom));
}

static void cmd_prbs(const char *rest)
{
   uint32_t seed  = 0;
   uint32_t count = 0;
   if (!parse_seed_count(rest, &seed, &count))
      return;
   if (current_master)
      op_prbs_tx(seed, count);
   else
      op_prbs(seed, count);
}

static void cmd_prbs_rx_master(const char *rest)
{
   if (!current_master) {
      printf("ERR V op is master-only\r\n");
      return;
   }
   uint32_t seed  = 0;
   uint32_t count = 0;
   if (!parse_seed_count(rest, &seed, &count))
      return;
   op_prbs_rx_master(seed, count);
}

static void cmd_prbs_xfer(const char *rest)
{
   if (!current_master) {
      printf("ERR X op is master-only\r\n");
      return;
   }
   uint32_t seed  = 0;
   uint32_t count = 0;
   if (!parse_seed_count(rest, &seed, &count))
      return;
   op_prbs_xfer(seed, count);
}

static void cmd_prbs_dma(const char *rest)
{
   if (current_master) {
      printf("ERR D op is slave-only\r\n");
      return;
   }
   uint32_t seed  = 0;
   uint32_t count = 0;
   if (!parse_seed_count(rest, &seed, &count))
      return;
   op_prbs_dma(seed, count);
}

static void cmd_loop(const char *rest)
{
   uint32_t count = 0;
   (void)parse_u32(rest, &count);
   if (count == 0U || (count & 3U) != 0U) {
      printf("ERR count must be >0 and a multiple of 4\r\n");
      return;
   }
   if (current_master)
      op_tx_zeros(count);
   else
      op_loopback(count);
}

static void handle_command(const char *line)
{
   line   = skip_ws(line);
   char c = line[0];

   switch (c) {
      case 'R': cmd_role(line + 1); break;
      case 'M': cmd_mode(line + 1); break;
      case 'P': cmd_prbs(line + 1); break;
      case 'V': cmd_prbs_rx_master(line + 1); break;
      case 'X': cmd_prbs_xfer(line + 1); break;
      case 'D': cmd_prbs_dma(line + 1); break;
      case 'L': cmd_loop(line + 1); break;
      case '?':
         printf("STATE role=%s mode=%s clkdiv=%u\r\n",
                current_master ? "master" : "slave", miom_name(current_miom),
                current_clkdiv);
         break;
      default: printf("ERR unknown cmd '%c'\r\n", c); break;
   }
}

int main(void)
{
   static const struct clocks_cfg clk = BOARD_CLOCKS_CFG;
   clocks_init(&clk);
   uart_init(BOARD_BAUD_DIV);
   timer_init();
   delay_ms(STARTUP_MS);

   printf("\r\nqspi PRBS demo starting\r\n");

   pinmux_spi2();
   spi_base = REG_SPI0_BASE + ((uint32_t)SPI_PORT * SPI_STRIDE);
   spi_reconfigure(SPI_MIO_SINGLE, false, 0);

   printf("READY role=slave mode=%s\r\n", miom_name(current_miom));

   static char line[CMD_BUF_SIZE];
   for (;;) {
      uart_read_line(line, CMD_BUF_SIZE);
      handle_command(line);
   }
}
