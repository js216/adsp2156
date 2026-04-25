// SPDX-License-Identifier: MIT
// main.c --- SPI0 <-> SPI1 ping-pong DMA + PRBS + CRC-32 shell
// Copyright (c) 2026 Jakob Kastelic

// Full-duplex throughput tester for SPI0 / SPI1 on the
// EV-21569-SOM + EZLITE carrier.  Expects 4 jumpers on P14:
//
//   P14.02 (SPI0_CLK)  <--> P14.12 (SPI1_CLK)
//   P14.04 (SPI0_MISO) <--> P14.14 (SPI1_MISO)
//   P14.06 (SPI0_MOSI) <--> P14.16 (SPI1_MOSI)
//   P14.08 (SPI0_SSB)  <--> P14.18 (SPI1_SEL1*)
//
// Both directions run in ping-pong mode.  Each side owns a
// TX ring (two half-buffers, CPU refills halves with fresh
// PRBS as the DMA drains them) and an RX ring (DMA fills
// halves, CPU CRC-32's each completed half).  The test
// terminates after a user-selected total word count; the
// master SPI is disabled once its TX pass count hits the
// target so the bus goes quiet and both RX DMAs naturally
// drain the final half.
//
// PRBS + CRC-32: zlib-compatible CRC (poly 0xEDB88320, init /
// xorout 0xFFFFFFFF), same xorshift32 seed on both TX buffers
// so both directions carry the same stream.  The receive side
// rebuilds the expected stream locally (a second PRBS generator
// seeded identically) and folds each received word into a
// running CRC-32 for integrity; a separate byte-by-byte
// comparator feeds the mismatch counter.
//
// UART0 shell commands (CR or LF commits):
//   help | ?    print help + state
//   s <hex>     set PRBS seed (default 0xC0FFEE)
//   n <words>   set transfer length in 32-bit words per side
//   c <int>     master SPI_CLK divisor (default 4 ~ 23 MHz)
//   a           run test A: SPI0 master, SPI1 slave
//   b           run test B: SPI1 master, SPI0 slave
//
// SPI0 pinmux reclaims PA6/PA7 from UART0, so printf is silent
// for the duration of the run.  The report lands after UART0
// is handed back via pinmux_uart0().

#include "board.h"
#include "clocks.h"
#include "dma.h"
#include "pinmux.h"
#include "regs.h"
#include "spi.h"
#include "timer.h"
#include "uart.h"
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

// ----- sizes -----
#define KIB              1024U
#define KIB_64           1024ULL
#define WORDS_PER_KIB    (KIB / 4U)
#define HALF_KIB         32U
#define HALF_WORDS       (HALF_KIB * WORDS_PER_KIB) // 32 KiB per half
#define BUF_WORDS        (2U * HALF_WORDS)          // 64 KiB per ring
#define DEFAULT_SEED     0xC0FFEEU
#define DEFAULT_BYTES    (256ULL * KIB_64) // 256 KiB per direction
#define DEFAULT_NWORDS   (DEFAULT_BYTES / 4ULL)
#define DEFAULT_CLKDIV   4U
#define CMD_BUF_SIZE     64U
#define BYTES_PER_WORD   4U
#define BITS_PER_BYTE    8U
#define BYTE_MASK        0xFFU
#define SPI_STRIDE_BYTES 0x1000U

// Byte-shift positions for MSB-first packing of 32-bit words.
#define BYTE3_SHIFT (3U * BITS_PER_BYTE)
#define BYTE2_SHIFT (2U * BITS_PER_BYTE)
#define BYTE1_SHIFT BITS_PER_BYTE

// Decimal/hex parser radices.
#define BASE_DEC 10U
#define BASE_HEX 16U
// Hex letter offset: 'a'..'f' -> 10..15.
#define HEX_LETTER_OFFSET 10U
// ASCII upper->lower delta (for case-insensitive suffix match).
#define ASCII_CASE_DELTA 32

// 64-bit helpers.
#define DEC_RADIX_64     10ULL
#define MS_PER_S_64      1000ULL
#define BITS_PER_BYTE_64 8ULL
#define U64_BITS         64

// Stall watchdog: 5 s with no RX progress -> bail.
#define STALL_TIMEOUT_S 5U

// Throughput print: leading-zero thresholds for 3-digit fraction.
#define FRAC_LT_100 100ULL
#define FRAC_LT_10  10ULL

// Backspace key code (ASCII DEL).
#define KEY_DEL 0x7F

// Two TX rings + two RX rings, one pair per SPI block.  4 rings
// at 64 KiB each = 256 KiB, fits L2 easily.
#pragma section("seg_l2_bss", NO_INIT)
static uint32_t ring_m_tx[BUF_WORDS];
#pragma section("seg_l2_bss", NO_INIT)
static uint32_t ring_m_rx[BUF_WORDS];
#pragma section("seg_l2_bss", NO_INIT)
static uint32_t ring_s_tx[BUF_WORDS];
#pragma section("seg_l2_bss", NO_INIT)
static uint32_t ring_s_rx[BUF_WORDS];

// Ping-pong descriptor pairs (one per DMA channel in use).
#pragma section("seg_l2_bss", NO_INIT)
static struct dma_dscl desc_m_tx[2];
#pragma section("seg_l2_bss", NO_INIT)
static struct dma_dscl desc_m_rx[2];
#pragma section("seg_l2_bss", NO_INIT)
static struct dma_dscl desc_s_tx[2];
#pragma section("seg_l2_bss", NO_INIT)
static struct dma_dscl desc_s_rx[2];

// ----- diag -----
static void diag_puts(const char *s)
{
   while (*s)
      uart_putc(*s++);
}

#define DIAG_NIBBLE_MASK   0xFU
#define DIAG_NIBBLE_BITS   4U
#define DIAG_HEX_TOP_SHIFT 28
#define DIAG_DEC_RADIX     10U

static void diag_hex32(uint32_t v)
{
   for (int shift = DIAG_HEX_TOP_SHIFT; shift >= 0;
        shift -= (int)DIAG_NIBBLE_BITS) {
      uint32_t n = (v >> (unsigned)shift) & DIAG_NIBBLE_MASK;
      uart_putc(
          (char)(n < DIAG_DEC_RADIX ? '0' + n : 'a' + (n - DIAG_DEC_RADIX)));
   }
}

static void diag_dec(uint32_t v)
{
   static const uint32_t pow10[] = {
       1000000000U, 100000000U, 10000000U, 1000000U, 100000U,
       10000U,      1000U,      100U,      10U,      1U};
   bool emitted = false;
   for (unsigned i = 0; i < sizeof(pow10) / sizeof(pow10[0]); i++) {
      uint32_t digit = 0;
      while (v >= pow10[i]) {
         v -= pow10[i];
         digit++;
      }
      if (digit != 0 || emitted || pow10[i] == 1U) {
         uart_putc((char)('0' + digit));
         emitted = true;
      }
   }
}

// ----- PRBS xorshift32 -----
#define PRBS_TAP_A 13U
#define PRBS_TAP_B 17U
#define PRBS_TAP_C 5U

static inline uint32_t prbs_step(uint32_t x)
{
   x ^= (x << PRBS_TAP_A);
   x ^= (x >> PRBS_TAP_B);
   x ^= (x << PRBS_TAP_C);
   return x;
}

// Fill ``nwords`` words with PRBS bytes, MSB-first packing so
// byte 0 of the stream lands in the top byte of buf[0].  Starting
// ``*x`` updated so successive calls continue the stream.
static uint32_t prbs_fill(uint32_t *buf, uint32_t nwords, uint32_t x)
{
   if (x == 0U)
      x = 1U;
   for (uint32_t i = 0; i < nwords; i++) {
      uint32_t w = 0;
      for (unsigned k = 0; k < BYTES_PER_WORD; k++) {
         x = prbs_step(x);
         w = (w << BITS_PER_BYTE) | (x & BYTE_MASK);
      }
      buf[i] = w;
   }
   return x;
}

// ----- CRC-32 (zlib-compatible, reflected) -----
#define CRC32_INIT   0xFFFFFFFFU
#define CRC32_XOROUT 0xFFFFFFFFU
#define CRC32_POLY   0xEDB88320U

static inline uint32_t crc32_byte(uint32_t c, uint8_t b)
{
   c ^= b;
   for (unsigned i = 0; i < BITS_PER_BYTE; i++) {
      uint32_t mask = (uint32_t)0 - (c & 1U);
      c             = (c >> 1U) ^ (CRC32_POLY & mask);
   }
   return c;
}

static uint32_t crc32_update(uint32_t c, const uint32_t *buf, uint32_t nwords)
{
   for (uint32_t i = 0; i < nwords; i++) {
      uint32_t w = buf[i];
      c          = crc32_byte(c, (uint8_t)(w >> BYTE3_SHIFT));
      c          = crc32_byte(c, (uint8_t)(w >> BYTE2_SHIFT));
      c          = crc32_byte(c, (uint8_t)(w >> BYTE1_SHIFT));
      c          = crc32_byte(c, (uint8_t)(w));
   }
   return c;
}

// ----- SPI helpers -----
static uint32_t spi_base(enum spi_id id)
{
   return REG_SPI0_BASE + ((uint32_t)id * SPI_STRIDE_BYTES);
}

static enum dma_channel spi_tx_chan(enum spi_id id)
{
   return (id == SPI_ID_0) ? DMA_CH_SPI0_TX : DMA_CH_SPI1_TX;
}

static enum dma_channel spi_rx_chan(enum spi_id id)
{
   return (id == SPI_ID_0) ? DMA_CH_SPI0_RX : DMA_CH_SPI1_RX;
}

static void setup_pair(enum spi_id master, enum spi_id slave, uint32_t clkdiv)
{
   struct spi_cfg m = {
       .clkdiv    = clkdiv,
       .size      = SPI_WORD_32,
       .miom      = SPI_MIO_SINGLE,
       .is_master = 1U,
       .cpol      = 0,
       .cpha      = 1U,
       .lsb_first = 0,
   };
   struct spi_cfg s = m;
   s.is_master      = 0U;
   s.clkdiv         = 0U;

   spi_disable(master);
   spi_disable(slave);
   if (master == SPI_ID_0)
      pinmux_spi0(1);
   else
      pinmux_spi1(1);
   if (slave == SPI_ID_0)
      pinmux_spi0(0);
   else
      pinmux_spi1(0);
   spi_init(master, &m);
   spi_init(slave, &s);
   spi_tx_dma_enable(master);
   spi_rx_dma_enable(master);
   spi_tx_dma_enable(slave);
   spi_rx_dma_enable(slave);
}

static void release_pair(enum spi_id master, enum spi_id slave)
{
   spi_disable(master);
   spi_disable(slave);
   pinmux_uart0();
}

// ----- test runner (ping-pong streaming) -----
static uint32_t g_seed   = DEFAULT_SEED;
static uint64_t g_nwords = DEFAULT_NWORDS;
static uint32_t g_clkdiv = DEFAULT_CLKDIV;

struct pp_state {
   uint32_t *ring;
   struct dma_dscl *desc;
   enum dma_channel ch;
   uint32_t next_half;  // which half will be consumed next (0 or 1)
   uint32_t prbs_state; // for TX refill or RX verify
   uint32_t crc;
   uint64_t words_done;
   uint64_t mismatches;
   uint64_t first_mismatch_word; // 1-based absolute word index
};

// TX: on each half the DMA has finished reading from, overwrite
// it with the next PRBS run so the far side keeps receiving a
// coherent stream.  Call from the wrap handler for TX channels.
static void pp_tx_refill(struct pp_state *tx)
{
   uint32_t *half = tx->ring + ((size_t)tx->next_half * (size_t)HALF_WORDS);
   tx->prbs_state = prbs_fill(half, HALF_WORDS, tx->prbs_state);
   tx->next_half ^= 1U;
   tx->words_done += (uint64_t)HALF_WORDS;
}

// RX: on each half the DMA has finished writing to, fold its
// contents into the running CRC and diff against the expected
// PRBS stream for mismatch counting.
// Returns true on first mismatch found in this half (caller then
// tears the test down immediately).  first_mismatch_word is the
// 1-based absolute word index of the offending word.
static bool pp_rx_verify(struct pp_state *rx)
{
   const volatile uint32_t *half =
       (const volatile uint32_t *)(rx->ring + ((size_t)rx->next_half *
                                               (size_t)HALF_WORDS));
   uint32_t local_mism = 0;
   uint32_t crc        = rx->crc;
   uint32_t x          = rx->prbs_state;
   uint64_t first_bad  = 0;
   uint64_t base_word  = rx->words_done;
   for (uint32_t i = 0; i < HALF_WORDS; i++) {
      uint32_t got = half[i];
      uint32_t w   = 0;
      for (unsigned k = 0; k < BYTES_PER_WORD; k++) {
         x = prbs_step(x);
         w = (w << BITS_PER_BYTE) | (x & BYTE_MASK);
      }
      if (got != w) {
         local_mism++;
         if (first_bad == 0ULL)
            first_bad = base_word + (uint64_t)i + 1ULL;
      }
      crc = crc32_byte(crc, (uint8_t)(got >> BYTE3_SHIFT));
      crc = crc32_byte(crc, (uint8_t)(got >> BYTE2_SHIFT));
      crc = crc32_byte(crc, (uint8_t)(got >> BYTE1_SHIFT));
      crc = crc32_byte(crc, (uint8_t)got);
   }
   rx->mismatches += local_mism;
   rx->crc        = crc;
   rx->prbs_state = x;
   rx->next_half ^= 1U;
   rx->words_done += (uint64_t)HALF_WORDS;
   if (first_bad != 0ULL) {
      rx->first_mismatch_word = first_bad;
      return true;
   }
   return false;
}

// ----- 64-bit helpers (cc21k -no-std-lib lacks __udivdi3) -----
// Binary long-division, 64 iterations worst case.  Slow but
// we only run it a few times per test.
static uint64_t udiv64(uint64_t n, uint64_t d)
{
   if (d == 0ULL)
      return 0ULL;
   uint64_t q = 0ULL;
   uint64_t r = 0ULL;
   for (unsigned bit = U64_BITS; bit-- > 0U;) {
      r = (r << 1U) | ((n >> bit) & 1ULL);
      if (r >= d) {
         r -= d;
         q |= ((uint64_t)1ULL << bit);
      }
   }
   return q;
}

// Iterative base-10 print of a uint64.  Buffered so a recursive
// /10 chain is unnecessary -- the longest unsigned 64-bit value
// is 20 digits, +1 for NUL termination.
#define DEC64_BUF_LEN 21U

static void diag_dec64(uint64_t v)
{
   char buf[DEC64_BUF_LEN];
   unsigned n = 0U;
   if (v == 0ULL) {
      uart_putc('0');
      return;
   }
   while (v > 0ULL && n < DEC64_BUF_LEN) {
      uint64_t q     = udiv64(v, DEC_RADIX_64);
      uint32_t digit = (uint32_t)(v - (q * DEC_RADIX_64));
      buf[n++]       = (char)('0' + digit);
      v              = q;
   }
   while (n-- > 0U)
      uart_putc(buf[n]);
}

#define U64_HI32_SHIFT 32U

static void diag_hex64(uint64_t v)
{
   diag_hex32((uint32_t)(v >> U64_HI32_SHIFT));
   diag_hex32((uint32_t)v);
}

#define HZ_PER_MHZ   1000000U
#define TICKS_PER_US (BOARD_SCLK_HZ / HZ_PER_MHZ)

// bytes:  total bytes transferred (uint64)
// ticks:  total elapsed SCLK0 ticks (uint64 accumulator)
static void print_rate(const char *label, uint64_t bytes, uint64_t ticks)
{
   uint64_t us = udiv64(ticks, (uint64_t)TICKS_PER_US);
   if (us == 0ULL)
      us = 1ULL;
   uint64_t bits = bytes * BITS_PER_BYTE_64;
   // Mbps * 1000 = bits * 1000 / us; split whole + milli frac.
   uint64_t whole = udiv64(bits, us);
   uint64_t rem   = bits - (whole * us);
   uint64_t frac  = udiv64(rem * MS_PER_S_64, us);

   diag_puts(label);
   diag_dec64(bytes);
   diag_puts(" B in ");
   diag_dec64(us);
   diag_puts(" us (");
   diag_dec64(whole);
   diag_puts(".");
   if (frac < FRAC_LT_100)
      diag_puts("0");
   if (frac < FRAC_LT_10)
      diag_puts("0");
   diag_dec64(frac);
   diag_puts(" Mbps)\r\n");
}

struct run_outcome {
   bool jumper_timeout;
   bool bit_error;
   bool error_on_master;
   uint64_t total_ticks;
};

// Iterate the four DMA channels, draining RX halves into running
// CRC-32 and topping up TX halves as they retire.  Bails on the
// first bit-error or stall.  Pulled out of run_test() so the main
// entry point stays under the cognitive-complexity gate.
static void run_drain_loop(struct pp_state *tx_m, struct pp_state *tx_s,
                           struct pp_state *rx_m, struct pp_state *rx_s,
                           struct run_outcome *out)
{
   uint32_t last_tick = timer_ticks();
   uint32_t deadline  = last_tick + (BOARD_SCLK_HZ * STALL_TIMEOUT_S);
   while (rx_m->words_done < g_nwords || rx_s->words_done < g_nwords) {
      uint32_t now = timer_ticks();
      out->total_ticks += (uint64_t)(now - last_tick);
      last_tick = now;

      if (dma_wrap_check(rx_m->ch) && rx_m->words_done < g_nwords) {
         if (pp_rx_verify(rx_m)) {
            out->bit_error       = true;
            out->error_on_master = true;
            return;
         }
         deadline = now + (BOARD_SCLK_HZ * STALL_TIMEOUT_S);
      }
      if (dma_wrap_check(rx_s->ch) && rx_s->words_done < g_nwords) {
         if (pp_rx_verify(rx_s)) {
            out->bit_error       = true;
            out->error_on_master = false;
            return;
         }
         deadline = now + (BOARD_SCLK_HZ * STALL_TIMEOUT_S);
      }
      if (dma_wrap_check(tx_m->ch) &&
          tx_m->words_done < g_nwords + (uint64_t)BUF_WORDS)
         pp_tx_refill(tx_m);
      if (dma_wrap_check(tx_s->ch) &&
          tx_s->words_done < g_nwords + (uint64_t)BUF_WORDS)
         pp_tx_refill(tx_s);

      // Stall: signed compare on wrapping u32.
      if ((int32_t)(now - deadline) > 0) {
         out->jumper_timeout = true;
         return;
      }
   }
   uint32_t now = timer_ticks();
   out->total_ticks += (uint64_t)(now - last_tick);
}

static void print_test_report(const struct pp_state *rx_m,
                              const struct pp_state *rx_s,
                              const struct run_outcome *out)
{
   diag_puts("master_rx words=");
   diag_dec64(rx_m->words_done);
   diag_puts(" CRC=0x");
   diag_hex32(rx_m->crc ^ CRC32_XOROUT);
   diag_puts(" mism=");
   diag_dec64(rx_m->mismatches);
   diag_puts("\r\nslave_rx  words=");
   diag_dec64(rx_s->words_done);
   diag_puts(" CRC=0x");
   diag_hex32(rx_s->crc ^ CRC32_XOROUT);
   diag_puts(" mism=");
   diag_dec64(rx_s->mismatches);
   diag_puts("\r\n");

   if (out->bit_error) {
      const struct pp_state *e = out->error_on_master ? rx_m : rx_s;
      diag_puts("WARN bit-error on ");
      diag_puts(out->error_on_master ? "master_rx" : "slave_rx");
      diag_puts(" at word #");
      diag_dec64(e->first_mismatch_word);
      diag_puts(" -- aborting\r\n");
   } else if (out->jumper_timeout) {
      diag_puts("WARN no-progress timeout (5 s) -- check jumpers\r\n");
   } else {
      diag_puts("PASS\r\n");
   }

   uint64_t bytes_m   = rx_m->words_done * (uint64_t)BYTES_PER_WORD;
   uint64_t bytes_s   = rx_s->words_done * (uint64_t)BYTES_PER_WORD;
   uint64_t bytes_min = (bytes_m < bytes_s) ? bytes_m : bytes_s;
   if (bytes_min > 0ULL || !out->bit_error)
      print_rate("throughput: ", bytes_min, out->total_ticks);
   diag_puts("master_rx_bytes=");
   diag_dec64(bytes_m);
   diag_puts(" slave_rx_bytes=");
   diag_dec64(bytes_s);
   diag_puts(" total_us=");
   diag_dec64(udiv64(out->total_ticks, (uint64_t)TICKS_PER_US));
   diag_puts("\r\n");
}

static void print_run_header(enum spi_id master, enum spi_id slave)
{
   diag_puts("test: master=SPI");
   diag_dec((uint32_t)master);
   diag_puts(" slave=SPI");
   diag_dec((uint32_t)slave);
   diag_puts(" n=");
   diag_dec64(g_nwords);
   diag_puts(" seed=0x");
   diag_hex32(g_seed);
   diag_puts(" clkdiv=");
   diag_dec(g_clkdiv);
   diag_puts("\r\n");
}

static void run_test(enum spi_id master, enum spi_id slave)
{
   if (g_nwords == 0ULL || (g_nwords % (uint64_t)HALF_WORDS) != 0ULL) {
      diag_puts("ERR n must be a non-zero multiple of ");
      diag_dec((uint32_t)HALF_WORDS);
      diag_puts(" words\r\n");
      return;
   }
   print_run_header(master, slave);

   struct pp_state tx_m = {.ring       = ring_m_tx,
                           .desc       = desc_m_tx,
                           .ch         = spi_tx_chan(master),
                           .next_half  = 0,
                           .prbs_state = g_seed ? g_seed : 1U,
                           .words_done = 0};
   struct pp_state tx_s = {.ring       = ring_s_tx,
                           .desc       = desc_s_tx,
                           .ch         = spi_tx_chan(slave),
                           .next_half  = 0,
                           .prbs_state = g_seed ? g_seed : 1U,
                           .words_done = 0};
   struct pp_state rx_m = {.ring                = ring_m_rx,
                           .desc                = desc_m_rx,
                           .ch                  = spi_rx_chan(master),
                           .next_half           = 0,
                           .prbs_state          = g_seed ? g_seed : 1U,
                           .crc                 = CRC32_INIT,
                           .words_done          = 0,
                           .mismatches          = 0,
                           .first_mismatch_word = 0};
   struct pp_state rx_s = {.ring                = ring_s_rx,
                           .desc                = desc_s_rx,
                           .ch                  = spi_rx_chan(slave),
                           .next_half           = 0,
                           .prbs_state          = g_seed ? g_seed : 1U,
                           .crc                 = CRC32_INIT,
                           .words_done          = 0,
                           .mismatches          = 0,
                           .first_mismatch_word = 0};

   // Pre-fill both halves of both TX rings.
   tx_m.prbs_state = prbs_fill(ring_m_tx, BUF_WORDS, tx_m.prbs_state);
   tx_s.prbs_state = prbs_fill(ring_s_tx, BUF_WORDS, tx_s.prbs_state);
   tx_m.words_done = (uint64_t)BUF_WORDS;
   tx_s.words_done = (uint64_t)BUF_WORDS;

   setup_pair(master, slave, g_clkdiv);

   dma_pingpong_rx_config(rx_m.ch, ring_m_rx, ring_m_rx + (size_t)HALF_WORDS,
                          HALF_WORDS, desc_m_rx);
   dma_pingpong_rx_config(rx_s.ch, ring_s_rx, ring_s_rx + (size_t)HALF_WORDS,
                          HALF_WORDS, desc_s_rx);
   dma_pingpong_tx_config(tx_s.ch, ring_s_tx, ring_s_tx + (size_t)HALF_WORDS,
                          HALF_WORDS, desc_s_tx);
   spi_tx_enable(slave);
   spi_rx_enable(slave);
   spi_rx_enable(master);

   dma_pingpong_tx_config(tx_m.ch, ring_m_tx, ring_m_tx + (size_t)HALF_WORDS,
                          HALF_WORDS, desc_m_tx);

   struct run_outcome out = {
       .jumper_timeout  = false,
       .bit_error       = false,
       .error_on_master = false,
       .total_ticks     = 0ULL,
   };
   spi_tx_enable(master);
   run_drain_loop(&tx_m, &tx_s, &rx_m, &rx_s, &out);

   // Cut the master; tear everything down; restore UART0.
   spi_disable(master);
   spi_disable(slave);
   dma_disable(tx_m.ch);
   dma_disable(tx_s.ch);
   dma_disable(rx_m.ch);
   dma_disable(rx_s.ch);
   release_pair(master, slave);

   print_test_report(&rx_m, &rx_s, &out);
   diag_puts("\r\n");
}

// ----- shell -----
static const char *skip_ws(const char *s)
{
   while (*s == ' ' || *s == '\t')
      s++;
   return s;
}

static const char *parse_u32(const char *s, uint32_t *out)
{
   s             = skip_ws(s);
   uint32_t v    = 0;
   bool got      = false;
   unsigned base = BASE_DEC;
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
         d = HEX_LETTER_OFFSET + (unsigned)(c - 'a');
      else if (base == BASE_HEX && c >= 'A' && c <= 'F')
         d = HEX_LETTER_OFFSET + (unsigned)(c - 'A');
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

// Parse decimal/hex into u64.  Optional case-insensitive suffix
// k/m/g/t multiplies by 1024^{1,2,3,4} so the user can say
// `n 1g` for 1 GiB (= 268435456 words at 4 B/word).  Suffix
// applies to the BYTE count, then the helper converts to words.
static const char *parse_u64_bytes(const char *s, uint64_t *out_words)
{
   s             = skip_ws(s);
   uint64_t v    = 0;
   bool got      = false;
   unsigned base = BASE_DEC;
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
         d = HEX_LETTER_OFFSET + (unsigned)(c - 'a');
      else if (base == BASE_HEX && c >= 'A' && c <= 'F')
         d = HEX_LETTER_OFFSET + (unsigned)(c - 'A');
      else
         break;
      v   = v * base + d;
      got = true;
      s++;
   }
   uint64_t mult = 1ULL;
   char suf      = *s;
   if (suf == 'k' || suf == 'K') {
      mult = KIB_64;
      s++;
   } else if (suf == 'm' || suf == 'M') {
      mult = KIB_64 * KIB_64;
      s++;
   } else if (suf == 'g' || suf == 'G') {
      mult = KIB_64 * KIB_64 * KIB_64;
      s++;
   } else if (suf == 't' || suf == 'T') {
      mult = KIB_64 * KIB_64 * KIB_64 * KIB_64;
      s++;
   }
   if (got)
      *out_words = (v * mult) / (uint64_t)BYTES_PER_WORD;
   return s;
}

static void uart_read_line(char *buf, uint32_t sz)
{
   uint32_t n = 0;
   buf[0]     = '\0';
   for (;;) {
      int c = uart_try_getc();
      if (c < 0)
         continue;
      if (c == '\r' || c == '\n') {
         uart_putc('\r');
         uart_putc('\n');
         buf[n] = '\0';
         return;
      }
      if ((c == '\b' || c == KEY_DEL) && n > 0) {
         n--;
         buf[n] = '\0';
         uart_putc('\b');
         uart_putc(' ');
         uart_putc('\b');
         continue;
      }
      if (n + 1 < sz) {
         buf[n++] = (char)c;
         uart_putc((char)c);
      }
   }
}

static void print_help(void)
{
   diag_puts("commands:\r\n"
             "  help | ?    print help + state\r\n"
             "  s <hex>     set PRBS seed\r\n"
             "  n <bytes>[k|m|g|t]   set bytes-per-side (binary suffixes)\r\n"
             "                       e.g. 'n 1g' = 1 GiB.  Bytes must be a\r\n"
             "                       multiple of ");
   diag_dec((uint32_t)(HALF_WORDS * BYTES_PER_WORD));
   diag_puts(".\r\n"
             "  c <int>     master SPI_CLK divisor\r\n"
             "  a           test A: SPI0 master + SPI1 slave\r\n"
             "  b           test B: SPI1 master + SPI0 slave\r\n");
   diag_puts("state: seed=0x");
   diag_hex32(g_seed);
   diag_puts(" n=");
   diag_dec64(g_nwords);
   diag_puts(" words (");
   diag_dec64(g_nwords * (uint64_t)BYTES_PER_WORD);
   diag_puts(" B) clkdiv=");
   diag_dec(g_clkdiv);
   diag_puts("\r\n");
}

static void handle_line(const char *line)
{
   line   = skip_ws(line);
   char c = line[0];
   if (c == '\0')
      return;
   if (c == '?' || (c == 'h' && line[1] == 'e')) {
      print_help();
      return;
   }
   const char *rest = line + 1;
   uint32_t v       = 0;
   switch (c) {
      case 's':
         (void)parse_u32(rest, &v);
         g_seed = v;
         diag_puts("seed=0x");
         diag_hex32(g_seed);
         diag_puts("\r\n");
         break;
      case 'n': {
         uint64_t w = 0;
         (void)parse_u64_bytes(rest, &w);
         g_nwords = w;
         diag_puts("n=");
         diag_dec64(g_nwords);
         diag_puts(" words (");
         diag_dec64(g_nwords * (uint64_t)BYTES_PER_WORD);
         diag_puts(" bytes)\r\n");
         break;
      }
      case 'c':
         (void)parse_u32(rest, &v);
         g_clkdiv = v;
         diag_puts("clkdiv=");
         diag_dec(g_clkdiv);
         diag_puts("\r\n");
         break;
      case 'a': run_test(SPI_ID_0, SPI_ID_1); break;
      case 'b': run_test(SPI_ID_1, SPI_ID_0); break;
      default:
         diag_puts("ERR unknown cmd\r\n");
         print_help();
         break;
   }
}

int main(void)
{
   static const struct clocks_cfg clk = BOARD_CLOCKS_CFG;
   clocks_init(&clk);
   uart_init(BOARD_BAUD_DIV);
   timer_init();
   board_som_init(0U);

   diag_puts("\r\nspi pingpong dma+crc shell starting\r\n");
   print_help();
   diag_puts("> ");

   static char line[CMD_BUF_SIZE];
   for (;;) {
      uart_read_line(line, CMD_BUF_SIZE);
      handle_line(line);
      diag_puts("> ");
   }
}
