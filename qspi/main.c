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

// DMA receive buffer size, in 32-bit words.  Used as a ring
// in FLOW=AUTO mode -- the DMA cycles over it forever while
// the CPU chases the write pointer. Sized to give the verify
// loop many SPI-word-times of slack against worst-case
// host-stall or cache-miss jitter: 64 KiB / 4 = 16384 words
// at 30 Mbit/s quad fills in ~17 ms, well above any realistic
// CPU stall in the verify hot path.
#define DMA_BUF_WORDS (64U * 1024U / BYTES_PER_WORD) // 64 KiB

// SPI2 TFIFO depth on the ADSP-21569 is 4 32-bit words.  In
// master mode the peripheral transmits whatever lands in the
// FIFO, drains to empty, and then becomes idle; subsequent
// pushes do not restart the clock until TS has seen a full
// 1->0 transition since the previous burst.  Pushing more than
// TFIFO_DEPTH words back-to-back without waiting for TS leaves
// TFF latched and the whole peripheral stalled.  Break long
// transmits into TFIFO_DEPTH-sized bursts and drain between
// each so TS does its toggle.
#define SPI_TFIFO_DEPTH 4U

// Land the ring in L2 SRAM via seg_l2_bss. L2 is visible on the
// system fabric and needs no L1 MP alias translation, and the
// stock 21569 boot ROM misbehaves past ~32 KB of initialised
// data in L1 block1 -- putting a 64 KiB initialised buffer
// there FAULTs the core at reset. NO_INIT also saves the boot
// loader from zero-filling at startup.
#pragma section("seg_l2_bss", NO_INIT)
static uint32_t dma_rx_buf[DMA_BUF_WORDS];

// Non-variadic diagnostic emitters.  printf() on cc21k has been
// observed to corrupt single-argument %x/%s slots (variadic ABI
// spill only materialises for >=2 args), so SPY register dumps
// route through these helpers and touch putchar directly.
static void diag_puts(const char *s)
{
   while (*s)
      uart_putc(*s++);
}

#define DIAG_NIBBLE_MASK   0xFU
#define DIAG_NIBBLE_BITS   4U
#define DIAG_HEX_TOP_SHIFT 28 // (sizeof(uint32_t) * BITS_PER_BYTE) - 4
#define DIAG_DEC_RADIX     10U
#define DIAG_DEC_DIGITS    10 // ceil(log10(2^32))
// One extra slot for a NUL terminator, not needed by this emitter
// but kept so the buffer is a conventional string-sized array.
#define DIAG_DEC_BUF 11

static void diag_hex32(uint32_t v)
{
   for (int shift = DIAG_HEX_TOP_SHIFT; shift >= 0;
        shift -= (int)DIAG_NIBBLE_BITS) {
      uint32_t n = (v >> (unsigned)shift) & DIAG_NIBBLE_MASK;
      uart_putc(
          (char)(n < DIAG_DEC_RADIX ? '0' + n : 'a' + (n - DIAG_DEC_RADIX)));
   }
}

static void diag_udec(uint32_t v)
{
   char buf[DIAG_DEC_BUF];
   int n = 0;
   if (v == 0U) {
      uart_putc('0');
      return;
   }
   while (v > 0U && n < DIAG_DEC_DIGITS) {
      uint32_t d = 0;
      uint32_t r = v;
      // Division-free: subtract powers of ten not needed here
      // since we build LSB-first. Use repeated subtraction by 10
      // to avoid pulling in __divrem_u32.
      uint32_t q = 0;
      while (r >= DIAG_DEC_RADIX) {
         r -= DIAG_DEC_RADIX;
         q++;
      }
      d        = r;
      buf[n++] = (char)('0' + d);
      v        = q;
   }
   while (n > 0)
      uart_putc(buf[--n]);
}

static void diag_kv_hex(const char *k, uint32_t v)
{
   diag_puts(k);
   diag_puts("=0x");
   diag_hex32(v);
   diag_puts("\r\n");
}

static void diag_kv_u(const char *k, uint32_t v)
{
   diag_puts(k);
   uart_putc('=');
   diag_udec(v);
   diag_puts("\r\n");
}

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
   // Board-level SPI convention is CPOL=0 / CPHA=1: the FT4222
   // master that clocks this port at test time (and at boot)
   // uses pyft4222's `Cpha.CLK_TRAILING` when the .qspi header
   // flags byte is 0 -- see test_serv/poller.py :: _cpol_cpha.
   // CLK_TRAILING is CPHA=1 in standard SPI terms (data latched
   // on the second clock edge). Match that on the slave or the
   // shift register samples on the wrong edge and the 32-bit
   // word boundary never completes, leaving the RX FIFO empty.
   struct spi_cfg cfg = {
       .clkdiv    = clkdiv,
       .size      = SPI_WORD_32,
       .miom      = miom,
       .is_master = master ? 1U : 0U,
       .cpol      = 0,
       // Board-wide SPI convention is CPHA=1 (standard-SPI sense:
       // data latched on the second / trailing clock edge when
       // CPOL=0). pyft4222 expresses this as Cpha.CLK_TRAILING,
       // which is what flags=0 selects in poller.py :: _cpol_cpha.
       // Both slave and master roles must use the same phase or
       // full-duplex breaks: FT4222-as-master + DSP-as-slave
       // worked because both sides agreed on CPHA=1, but
       // DSP-as-master + FT4222-as-slave (steps 7 / 8 / 9) failed
       // when the master ran CPHA=0 -- the FT4222 slave sampled
       // MISO half a bit late relative to what the DSP master
       // emitted on MOSI, and its latched MISO output to the DSP
       // was similarly skewed, so every received byte decoded as
       // garbage. Use CPHA=1 unconditionally for both roles.
       .cpha      = 1U,
       .lsb_first = 0,
   };
   // PA5 alt-function depends on master vs slave role, and the
   // slave data-lane FER programming depends on lane width, so
   // rerun the pinmux every time to cover runtime M1/M2/M4 and
   // Rs/Rm transitions.
   pinmux_spi2(master ? 1 : 0, (unsigned)miom);
   spi_init(SPI_PORT, &cfg);
   if (!master)
      spi_rx_dma_enable(SPI_PORT);
   // Enable the TX channel only when the DSP itself sources the
   // data stream. In slave dual/quad receive the peripheral's
   // TX shifter has no useful data to send -- pushing a TEN=1
   // there starts an idle-pattern drive onto MISO/D1 (PA0) once
   // the TX FIFO underruns, which on hardware shows up as lane
   // D1 reading stuck-at-1 on subsequent RX bytes (the slave's
   // PA0 driver fights the master's D1 output). Leave TEN=0 in
   // slave role so PA0 stays a pure input in dual/quad modes
   // and a deterministic zero-driver output in single mode.
   if (master)
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
   // Bring-up aid: echo every parsed line. Remove once the
   // command path is declared reliable.
   diag_puts("RX[0x");
   diag_hex32(n);
   diag_puts("]>");
   diag_puts(buf);
   diag_puts("<\r\n");
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
   // Explicit if-chain instead of switch: cc21k's jump-table lowering
   // of enum switches was producing a NULL-return path (observed as
   // "mode=(null)" on UART), which this form sidesteps.
   static const char s_x1[] = "x1";
   static const char s_x2[] = "x2";
   static const char s_x4[] = "x4";
   static const char s_qq[] = "??";
   unsigned v               = (unsigned)m;
   if (v == (unsigned)SPI_MIO_SINGLE)
      return s_x1;
   if (v == (unsigned)SPI_MIO_DUAL)
      return s_x2;
   if (v == (unsigned)SPI_MIO_QUAD)
      return s_x4;
   return s_qq;
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

   diag_puts("PRBS mode=");
   diag_puts(miom_name(current_miom));
   diag_puts(" seed=0x");
   diag_hex32(seed);
   diag_puts(" N=0x");
   diag_hex32(count);
   if (mismatches == 0 && !ror) {
      diag_puts(" OK");
   } else {
      diag_puts(" FAIL mismatches=0x");
      diag_hex32(mismatches);
      diag_puts(" first_at=0x");
      diag_hex32(first_at);
   }
   diag_puts(" ticks=0x");
   diag_hex32(elapsed);
   diag_puts(" ror=0x");
   diag_hex32(ror);
   diag_puts("\r\n");
}

// Build the expected 32-bit word from four successive PRBS
// bytes, laid out MSB-first to match how spi_rx assembles a
// word from the incoming byte stream (first byte -> bits
// [31:24]). Packing four byte-compares into one 32-bit XOR is
// what lets the verify loop keep up with ~30 Mbit/s quad RX:
// the common (no-mismatch) path is a single load + one XOR +
// one branch per word, instead of four byte compares.
static inline uint32_t prbs_next_word(uint32_t *state)
{
   uint32_t w = 0;
   w |= (uint32_t)prbs_next(state) << WORD_MSB_SHIFT;
   w |= (uint32_t)prbs_next(state) << (WORD_MSB_SHIFT - BITS_PER_BYTE);
   w |= (uint32_t)prbs_next(state) << (WORD_MSB_SHIFT - 2U * BITS_PER_BYTE);
   w |= (uint32_t)prbs_next(state);
   return w;
}

// Verify one 32-bit word received from SPI against the PRBS
// reference stream; update the rolling mismatch tally in place.
// The fast path is a single uint32_t XOR -- only on mismatch do
// we re-scan bytewise to record which byte indices disagree.
static inline void verify_word(uint32_t w, uint32_t *state,
                               uint32_t *mismatches, uint32_t *first_at,
                               uint32_t *idx)
{
   uint32_t exp_w = prbs_next_word(state);
   if (w == exp_w) {
      *idx += BYTES_PER_WORD;
      return;
   }
   for (unsigned b = 0; b < BYTES_PER_WORD; b++) {
      unsigned shift = WORD_MSB_SHIFT - (b * BITS_PER_BYTE);
      uint8_t got    = (uint8_t)(w >> shift);
      uint8_t exp    = (uint8_t)(exp_w >> shift);
      if (got != exp) {
         if (*mismatches == 0U)
            *first_at = *idx;
         (*mismatches)++;
      }
      (*idx)++;
   }
}

// PRBS verify using the SPI2_RX peripheral DMA in autobuffer /
// FLOW=AUTO mode. The DMA walks `dma_rx_buf` as a circular
// region and the CPU chases behind it, verifying words as they
// land. Because the DMA never stops while the CPU works -- no
// re-arm gap between chunks -- the SPI RX FIFO cannot overflow
// even under sustained quad-lane clocking. The legacy per-chunk
// one-shot arrangement drops bytes at ~30 Mbit/s because the
// verify loop takes longer than the 4-deep RFIFO can hold, so
// by the time the CPU returns to re-arm DMA27 the SPI slave
// has already ROR'd.
//
// The consumer index `rd_pos` and the producer index (derived
// from XCNT_CUR) live in modular arithmetic over DMA_BUF_WORDS.
// Each loop iteration consumes every word the DMA has written
// since the last sample.  If the CPU ever falls DMA_BUF_WORDS
// behind, ROR is unavoidable; the SPI status check at the end
// catches that case.
static void op_prbs_dma(uint32_t seed, uint32_t count)
{
   spi_rx_flush();

   uint32_t state      = seed ? seed : 1U;
   uint32_t mismatches = 0;
   uint32_t first_at   = 0;
   uint32_t idx        = 0;
   uint32_t total      = count / BYTES_PER_WORD;
   uint32_t consumed   = 0;
   uint32_t rd_pos     = 0; // next word to verify in dma_rx_buf[]
   // High-water mark of words produced by the DMA between two
   // consecutive polls. If this ever reaches DMA_BUF_WORDS the
   // ring has been overrun by at least one full lap and the
   // verify data is silently corrupt (the SPI peripheral itself
   // will not raise ROR in that case -- its RFIFO is being
   // drained, just into stale memory). Report as overrun.
   uint32_t hwm_new_words = 0;

   // FLOW=AUTO: XCNT reloads to DMA_BUF_WORDS and ADDR_CUR wraps
   // to ADDRSTART whenever XCNT_CUR hits zero. Sampling XCNT_CUR
   // is enough to compute the producer index modulo DMA_BUF_WORDS.
   dma_autobuffer_config(SPI_RX_DMA,
                         (struct dma_buf){dma_rx_buf, DMA_BUF_WORDS},
                         DMA_DIR_RX_TO_MEM);
   dma_enable(SPI_RX_DMA);

   uint32_t t0 = timer_ticks();
   // Previous XCNT_CUR sample. Starts at DMA_BUF_WORDS because
   // the channel has just been armed and has yet to transfer a
   // single word; the first sample will decrement from there.
   uint32_t prev_xcnt_cur = DMA_BUF_WORDS;

   while (consumed < total) {
      uint32_t cur = dma_xcnt_cur(SPI_RX_DMA);
      // Words the DMA has written since the previous sample.
      // Normal case: cur <= prev_xcnt_cur, delta = prev - cur.
      // Wrap case: cur > prev_xcnt_cur means XCNT_CUR reloaded
      // (one complete lap around the ring since last sample),
      // delta = prev + (DMA_BUF_WORDS - cur). If the CPU is so
      // far behind that multiple laps have elapsed we cannot
      // tell -- ROR will flag that as data loss below.
      uint32_t new_words = (cur <= prev_xcnt_cur)
                               ? (prev_xcnt_cur - cur)
                               : (prev_xcnt_cur + DMA_BUF_WORDS - cur);
      prev_xcnt_cur      = cur;
      if (new_words > hwm_new_words)
         hwm_new_words = new_words;

      uint32_t to_consume = total - consumed;
      if (new_words > to_consume)
         new_words = to_consume;

      for (uint32_t k = 0; k < new_words; k++) {
         verify_word(dma_rx_buf[rd_pos], &state, &mismatches, &first_at, &idx);
         rd_pos++;
         if (rd_pos == DMA_BUF_WORDS)
            rd_pos = 0;
      }
      consumed += new_words;
   }
   uint32_t elapsed = timer_ticks() - t0;

   dma_disable(SPI_RX_DMA);

   uint32_t stat = MMR(spi_base + OFF_SPI_STAT);
   unsigned ror  = (stat & BIT_SPI_STAT_ROR) ? 1U : 0U;
   MMR(spi_base + OFF_SPI_STAT) =
       BIT_SPI_STAT_ROR | BIT_SPI_STAT_TUR | BIT_SPI_STAT_TC;

   // Ring overrun: CPU polled infrequently enough that the DMA
   // wrote DMA_BUF_WORDS or more words between two samples.
   // Report explicitly so the FAIL line is not silently taken
   // for a pin/lane-level bug.
   unsigned overrun = (hwm_new_words >= DMA_BUF_WORDS) ? 1U : 0U;

   diag_puts("PRBSDMA mode=");
   diag_puts(miom_name(current_miom));
   diag_puts(" seed=0x");
   diag_hex32(seed);
   diag_puts(" N=0x");
   diag_hex32(count);
   if (mismatches == 0 && !ror && !overrun) {
      diag_puts(" OK");
   } else {
      diag_puts(" FAIL mismatches=0x");
      diag_hex32(mismatches);
      diag_puts(" first_at=0x");
      diag_hex32(first_at);
      diag_puts(" overrun=0x");
      diag_hex32(overrun);
      diag_puts(" hwm=0x");
      diag_hex32(hwm_new_words);
   }
   diag_puts(" ticks=0x");
   diag_hex32(elapsed);
   diag_puts(" ror=0x");
   diag_hex32(ror);
   diag_puts("\r\n");
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

   // Same chunked-refill pattern as op_tx_zeros: push up to
   // SPI_TFIFO_DEPTH words, drain via TS, repeat.  The peripheral
   // does not accept a refill before TS has toggled, so a simple
   // while (TFF) push loop stalls at the fifth word.
   uint32_t t0 = timer_ticks();
   uint32_t i  = 0;
   while (i < words) {
      uint32_t chunk =
          (words - i) > SPI_TFIFO_DEPTH ? SPI_TFIFO_DEPTH : (words - i);
      for (uint32_t j = 0; j < chunk; j++) {
         uint32_t w = 0;
         for (unsigned b = 0; b < BYTES_PER_WORD; b++) {
            unsigned shift = WORD_MSB_SHIFT - (b * BITS_PER_BYTE);
            w |= (uint32_t)prbs_next(&state) << shift;
         }
         MMR(spi_base + OFF_SPI_TFIFO) = w;
      }
      while (MMR(spi_base + OFF_SPI_STAT) & BIT_SPI_STAT_TS)
         ;
      i += chunk;
   }
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
   uint32_t i     = 0;
   while (i < words) {
      uint32_t chunk =
          (words - i) > SPI_TFIFO_DEPTH ? SPI_TFIFO_DEPTH : (words - i);
      for (uint32_t j = 0; j < chunk; j++)
         MMR(spi_base + OFF_SPI_TFIFO) = 0U;
      while (MMR(spi_base + OFF_SPI_STAT) & BIT_SPI_STAT_TS)
         ;
      i += chunk;
   }
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
      diag_puts("ROLE slave mode=");
      diag_puts(miom_name(current_miom));
      diag_puts("\r\n");
   } else if (rest[0] == 'm') {
      uint32_t clkdiv = 0;
      (void)parse_u32(rest + 1, &clkdiv);
      spi_reconfigure(current_miom, true, clkdiv);
      diag_puts("ROLE master clkdiv=0x");
      diag_hex32(clkdiv);
      diag_puts(" mode=");
      diag_puts(miom_name(current_miom));
      diag_puts("\r\n");
   } else {
      diag_puts("ERR bad role\r\n");
   }
}

static void cmd_mode(const char *rest)
{
   enum spi_miom m = SPI_MIO_SINGLE;
   switch (rest[0]) {
      case '1': m = SPI_MIO_SINGLE; break;
      case '2': m = SPI_MIO_DUAL; break;
      case '4': m = SPI_MIO_QUAD; break;
      default: diag_puts("ERR bad mode\r\n"); return;
   }
   spi_reconfigure(m, current_master, current_clkdiv);
   diag_puts("MODE ");
   diag_puts(miom_name(current_miom));
   diag_puts("\r\n");
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

   spi_base = REG_SPI0_BASE + ((uint32_t)SPI_PORT * SPI_STRIDE);
   spi_reconfigure(SPI_MIO_SINGLE, false, 0);

   diag_puts("READY role=slave mode=");
   diag_puts(miom_name(current_miom));
   diag_puts("\r\n");

   // SELFTEST: prove the SPI2 peripheral itself is alive by
   // running it as master and clocking 4 words out. In master
   // mode RX and TX are always full-duplex, so every TFIFO push
   // generates a corresponding RFIFO entry (MISO floating reads
   // as 0x00 or 0xFF). If TFIFO drains and RFIFO fills, the
   // peripheral and its clock are functional; if nothing
   // happens, SPI2 has no clock or is misrouted.
#define SELFTEST_CLKDIV     9U
#define SELFTEST_WORDS      4U
#define SELFTEST_PATTERN    0xA5A5A500U
#define SELFTEST_WAIT_MS    100U
#define SCLK_TICKS_PER_MSEC 93750U
   printf("SELFTEST: master TX/RX roundtrip\r\n");
   spi_reconfigure(SPI_MIO_SINGLE, true, SELFTEST_CLKDIV);
   spi_rx_flush();
   for (uint32_t i = 0; i < SELFTEST_WORDS; i++) {
      while (MMR(spi_base + OFF_SPI_STAT) & BIT_SPI_STAT_TFF)
         ;
      MMR(spi_base + OFF_SPI_TFIFO) = SELFTEST_PATTERN | i;
   }
   while (MMR(spi_base + OFF_SPI_STAT) & BIT_SPI_STAT_TS)
      ;
   for (uint32_t i = 0; i < SELFTEST_WORDS; i++) {
      uint32_t t0 = timer_ticks();
      while ((MMR(spi_base + OFF_SPI_STAT) & BIT_SPI_STAT_RFE) &&
             (timer_ticks() - t0) < (SELFTEST_WAIT_MS * SCLK_TICKS_PER_MSEC)) {
      }
      uint32_t s = MMR(spi_base + OFF_SPI_STAT);
      if (s & BIT_SPI_STAT_RFE) {
         diag_puts("SELFTEST RFE stuck i=0x");
         diag_hex32(i);
         diag_puts(" stat=0x");
         diag_hex32(s);
         diag_puts("\r\n");
         break;
      }
      uint32_t w = MMR(spi_base + OFF_SPI_RFIFO);
      diag_puts("SELFTEST i=0x");
      diag_hex32(i);
      diag_puts(" rx=0x");
      diag_hex32(w);
      diag_puts("\r\n");
   }
   // Back to slave for the SPY loop.
   spi_reconfigure(SPI_MIO_SINGLE, false, 0);
   printf("SELFTEST done, back to slave\r\n");

   // SPY loop (Step 0.5 bring-up scaffold): poll SPI_STAT for
   // incoming RX words and sample PORTA_DATA so we can tell from
   // outside whether the external master is actually driving the
   // SPI2 pin group at all. This loop blocks the UART command loop
   // for its window, so it is gated behind SPY_ENABLE -- keep it
   // compiled in only when bringing up slave-role; disable it
   // (SPY_ENABLE = 0) when running Steps 5+ so the command path
   // starts responding immediately.
#define SPY_ENABLE 0
#if SPY_ENABLE
   // Enable input-enable on PA0..PA5 so PORTA_DATA reflects the live
   // pin state even while the pins are in alt-function mode; the PADs
   // input cell is shared with the SPI block so this is non-invasive.
#define PA_SPI2_MASK 0x0000003FU // PA0..PA5
   MMR(REG_PORTA_INEN_SET) = PA_SPI2_MASK;

   // Diagnostic: disable PORTA_FER for PA0..PA5 so the pins are in
   // pure GPIO input mode during the SPY window. That way
   // PORTA_DATA read-back definitely reflects the live pin state
   // rather than a peripheral-controlled shadow. The SPI
   // peripheral still has its pad input connected so SPI_STAT
   // behavior is unchanged, but now pin_or/pin_and actually
   // capture external activity from FT4222 regardless of how
   // the SPI block gates its internal SCLK receiver.
   //
   // Undone after the SPY window so normal operation resumes.
#define SPY_FER_ENTER_GPIO 0
#if SPY_FER_ENTER_GPIO
   uint32_t saved_fer = MMR(REG_PORTA_FER);
   MMR(REG_PORTA_FER) = saved_fer & ~PA_SPI2_MASK;
#endif

#define SPY_WINDOW_TICKS (700U * SCLK_TICKS_PER_MSEC) // 0.7 s
#define SPY_STAT_LOG_CAP 64U
#define SPY_WORD_CAP     32U // 16 words per 64-byte burst + slack
   diag_kv_hex("SPY SPI_STAT", MMR(spi_base + OFF_SPI_STAT));
   diag_kv_hex("SPY PORTA_DATA", MMR(REG_PORTA_DATA) & PA_SPI2_MASK);
   diag_kv_hex("SPY SPI_CTL", MMR(spi_base + OFF_SPI_CTL));
   diag_kv_hex("SPY SPI_TXCTL", MMR(spi_base + OFF_SPI_TXCTL));
   diag_kv_hex("SPY SPI_RXCTL", MMR(spi_base + OFF_SPI_RXCTL));
   diag_kv_hex("SPY SPI_SLVSEL", MMR(spi_base + OFF_SPI_SLVSEL));
   diag_kv_hex("SPY PORTA_MUX", MMR(REG_PORTA_MUX));
   diag_kv_hex("SPY PORTA_FER", MMR(REG_PORTA_FER));
   diag_kv_hex("SPY OSPI0_CTL", MMR(REG_OSPI0_CTL));
   diag_kv_hex("SPY SCB5_REMAP", MMR(REG_SCB5_REMAP));
   {
      // Drain RFIFO into an in-memory buffer during the active
      // window; print the buffer only after the window closes.
      // UART printf inside the polling loop is far slower than
      // the FT4222 master can fill the 2-deep 32-bit RFIFO (one
      // line is ~1.7 ms at 115200 baud; 16 words at 7.5 Mbit/s
      // SCLK arrive in ~70 us), so inline printing would ROR
      // the FIFO and drop most of the burst.
      static uint32_t word_buf[SPY_WORD_CAP];
      uint32_t nwords      = 0;
      uint32_t pin_or      = 0;
      uint32_t pin_and     = PA_SPI2_MASK;
      uint32_t pin_samples = 0;
      uint32_t t0          = timer_ticks();
      uint32_t stat_first  = MMR(spi_base + OFF_SPI_STAT);
      while ((timer_ticks() - t0) < SPY_WINDOW_TICKS) {
         uint32_t stat = MMR(spi_base + OFF_SPI_STAT);
         if (!(stat & BIT_SPI_STAT_RFE)) {
            uint32_t w = MMR(spi_base + OFF_SPI_RFIFO);
            if (nwords < SPY_WORD_CAP)
               word_buf[nwords] = w;
            nwords++;
         }
         uint32_t d = MMR(REG_PORTA_DATA) & PA_SPI2_MASK;
         pin_or |= d;
         pin_and &= d;
         pin_samples++;
      }
      uint32_t stat_final = MMR(spi_base + OFF_SPI_STAT);
      diag_kv_hex("SPY nwords", nwords);
      diag_kv_hex("SPY samples", pin_samples);
      diag_kv_hex("SPY pin_or", pin_or);
      diag_kv_hex("SPY pin_and", pin_and);
      diag_kv_hex("SPY stat_first", stat_first);
      diag_kv_hex("SPY stat_final", stat_final);
      diag_kv_hex("SPY ror", (stat_final & BIT_SPI_STAT_ROR) ? 1U : 0U);
      uint32_t n_print = nwords < SPY_WORD_CAP ? nwords : SPY_WORD_CAP;
      for (uint32_t i = 0; i < n_print; i++)
         diag_kv_hex("SPY w", word_buf[i]);
      // W1C the latched error bits so the command loop starts
      // from a clean STAT.
      MMR(spi_base + OFF_SPI_STAT) =
          BIT_SPI_STAT_ROR | BIT_SPI_STAT_TUR | BIT_SPI_STAT_TC;
   }
#if SPY_FER_ENTER_GPIO
   MMR(REG_PORTA_FER) = saved_fer;
#endif
#endif // SPY_ENABLE

   static char line[CMD_BUF_SIZE];
   for (;;) {
      uart_read_line(line, CMD_BUF_SIZE);
      handle_command(line);
   }
}
