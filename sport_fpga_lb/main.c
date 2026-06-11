// SPDX-License-Identifier: MIT
// main.c --- SPORT4 internal SRU loopback, polled, scaled to 2 GiB
// Copyright (c) 2026 Jakob Kastelic
//
// SPORT4 half-A (master TX) is routed via the DAI1 SRU crossbar
// to SPORT4 half-B (slave RX) -- no external pins, no FPGA. DSP
// CGU is reprogrammed to SCLK0 = 62.5 MHz; CLKDIV=0 ->
// bit_clk = 62.5 MHz exactly, matching both
// fSPTCLKPROG TX = 62.5 MHz and fSPTCLKEXT RX = 62.5 MHz
// (ADSP-2156x Table 19).
//
// Both halves are driven by polled reads and writes. Each loop
// iteration reads one word from SPORT_HALF_B, advances the
// expected LFSR, compares bit-for-bit, then writes the next
// LFSR word into SPORT_HALF_A's TX FIFO. The loop self-rate-
// limits to the SPORT word rate (62.5 MHz / 32 = ~1.95 MHz).
//
// TOTAL_BYTES / TOTAL_WORDS overridden at compile time so the
// Makefile can build 64 / 512 / 2048 MiB variants.
//
// DMA was attempted (dma_pingpong_tx_config and
// dma_autobuffer_config with DMA_CH_SPORT4_A) but consistently
// produced exactly 1 transmitted word at the FPGA before the
// SPORT stalled (FIFO under-feed). Polled hits the same
// SPORT-clock-limited throughput and is reliable.

#include "board.h"
#include "clocks.h"
#include "regs.h"
#include "sport.h"
#include "timer.h"
#include "uart.h"
#include <stdbool.h>
#include <stdint.h>

#ifndef TOTAL_BYTES
#define TOTAL_BYTES 67108864ULL
#endif
#ifndef TOTAL_WORDS
#define TOTAL_WORDS 16777216U
#endif

#define LFSR_SEED         0x12345678U
#define MAX_STARTUP_SKIP  4U
#define SPORT_WORD_BITS   32U
#define SPORT_FSDIV       31U
#define SPORT_SCLK_HZ     59375000U
#define SPORT_CLKDIV      0U
#define SPORT_BIT_CLK_HZ  59375000U
#define LOCAL_BAUD_DIV    ((SPORT_SCLK_HZ + (BOARD_BAUD / 2U)) / BOARD_BAUD)
#define BITS_PER_BYTE_64  8ULL
#define MIN_RATE_BPS      30000000ULL
#define SPORT4A_ERR_REG   0x31002420U
#define SPORT4B_ERR_REG   0x310024A0U
#define WAIT_TIMEOUT      10000000U
#define TICK_SAMPLE_MASK  0x3FFU

static void put_str(const char *s)
{
   while (*s != '\0')
      uart_putc(*s++);
}

static void put_u32(uint32_t v)
{
   static const uint32_t pow10[10] = {
       1000000000U, 100000000U, 10000000U, 1000000U, 100000U,
       10000U,      1000U,      100U,      10U,      1U,
   };
   if (v == 0U) {
      uart_putc('0');
      return;
   }
   unsigned started = 0U;
   for (unsigned i = 0U; i < 10U; i++) {
      uint32_t d = pow10[i];
      char c = '0';
      while (v >= d) {
         v -= d;
         c = (char)(c + 1);
      }
      if (c != '0')
         started = 1U;
      if (started != 0U)
         uart_putc(c);
   }
}

static void put_i32(int32_t v)
{
   if (v < 0) {
      uart_putc('-');
      put_u32((uint32_t)(-v));
   } else {
      put_u32((uint32_t)v);
   }
}

static uint64_t udiv64(uint64_t n, uint64_t d)
{
   uint64_t q = 0ULL;
   uint64_t r = 0ULL;
   for (int bit = 63; bit >= 0; bit--) {
      r = (r << 1) | ((n >> (unsigned)bit) & 1ULL);
      if (r >= d) {
         r -= d;
         q |= (1ULL << (unsigned)bit);
      }
   }
   return q;
}

static void put_u64(uint64_t v)
{
   static const uint64_t pow10[20] = {
       10000000000000000000ULL, 1000000000000000000ULL,
       100000000000000000ULL,   10000000000000000ULL,
       1000000000000000ULL,     100000000000000ULL,
       10000000000000ULL,       1000000000000ULL,
       100000000000ULL,         10000000000ULL,
       1000000000ULL,           100000000ULL,
       10000000ULL,             1000000ULL,
       100000ULL,               10000ULL,
       1000ULL,                 100ULL,
       10ULL,                   1ULL,
   };
   if (v == 0ULL) {
      uart_putc('0');
      return;
   }
   unsigned started = 0U;
   for (unsigned i = 0U; i < 20U; i++) {
      uint64_t d = pow10[i];
      char c = '0';
      while (v >= d) {
         v -= d;
         c = (char)(c + 1);
      }
      if (c != '0')
         started = 1U;
      if (started != 0U)
         uart_putc(c);
   }
}

static inline uint32_t lfsr_next(uint32_t *state)
{
   uint32_t s = *state;
   uint32_t bit =
       ((s >> 31) ^ (s >> 21) ^ (s >> 1) ^ (s >> 0)) & 1U;
   s = (s << 1) | bit;
   *state = s;
   return s;
}

static inline void accum_elapsed_ticks(uint64_t *elapsed_ticks,
                                       uint32_t *last_tick)
{
   uint32_t now = timer_ticks();
   *elapsed_ticks += (uint64_t)(uint32_t)(now - *last_tick);
   *last_tick = now;
}

static const struct sport_dsp_cfg tx_master_cfg = {
    .word_bits     = SPORT_WORD_BITS,
    .clkdiv        = SPORT_CLKDIV,
    .fsdiv         = SPORT_FSDIV,
    .is_tx         = true,
    .internal_clk  = true,
    .internal_fs   = true,
    .late_fs       = true,
    .data_indep_fs = false,
    .sample_rising = true,
};

static const struct sport_dsp_cfg rx_slave_cfg = {
    .word_bits     = SPORT_WORD_BITS,
    .clkdiv        = 0U,
    .fsdiv         = 0U,
    .is_tx         = false,
    .internal_clk  = false,
    .internal_fs   = false,
    .late_fs       = true,
    .data_indep_fs = false,
    .sample_rising = true,
};

int main(void)
{
   uart_init(BOARD_BAUD_DIV);
   while ((MMR(REG_UART0_STAT) & BIT_UART_STAT_THRE) == 0U) {
   }
   static const struct clocks_cfg clk = CLOCKS_CFG_SCLK0_59MHZ;
   clocks_init(&clk);
   MMR(REG_UART0_CLK) =
       BIT_UART_CLK_EDBO | (LOCAL_BAUD_DIV & MASK_UART_CLK_DIV);
   timer_init();
   board_som_init(0U);

   put_str("\r\nsport_fpga_lb boot\r\n");

   sport_install_internal_loopback(SPORT_ID_4);
   sport_dsp_serial_init(SPORT_ID_4, SPORT_HALF_B, &rx_slave_cfg);
   sport_dsp_serial_init(SPORT_ID_4, SPORT_HALF_A, &tx_master_cfg);
   sport_clear_errors(SPORT_ID_4, SPORT_HALF_A);
   sport_clear_errors(SPORT_ID_4, SPORT_HALF_B);

   uint32_t tx_state = LFSR_SEED;
   uint32_t rx_state = LFSR_SEED;

   // Preload two TX words so the FIFO has data the moment
   // half-A enables.
   sport_write_raw(SPORT_ID_4, SPORT_HALF_A, lfsr_next(&tx_state));
   sport_write_raw(SPORT_ID_4, SPORT_HALF_A, lfsr_next(&tx_state));

   uint32_t last_tick = timer_ticks();
   sport_enable(SPORT_ID_4, SPORT_HALF_B);
   sport_enable(SPORT_ID_4, SPORT_HALF_A);

   uint64_t elapsed_ticks = 0ULL;
   uint32_t sent = 2U;
   uint32_t got = 0U;
   uint32_t errors = 0U;
   int32_t firsterr = -1;
   uint32_t rx_timeouts = 0U;
   uint32_t tx_timeouts = 0U;
   uint32_t skipped = 0U;
   bool synced = false;

   for (uint32_t i = 0U; i < TOTAL_WORDS; i++) {
      if ((i & TICK_SAMPLE_MASK) == 0U)
         accum_elapsed_ticks(&elapsed_ticks, &last_tick);

      uint32_t w = 0U;
      uint32_t rxp = 0U;
      while (!sport_rx_ready(SPORT_ID_4, SPORT_HALF_B)) {
         if (++rxp > WAIT_TIMEOUT) {
            rx_timeouts++;
            break;
         }
      }
      if (rx_timeouts != 0U)
         break;
      w = sport_read_raw(SPORT_ID_4, SPORT_HALF_B);
      got++;

      uint32_t exp = lfsr_next(&rx_state);
      if (!synced) {
         while (exp != w && skipped < MAX_STARTUP_SKIP) {
            exp = lfsr_next(&rx_state);
            skipped++;
         }
         if (exp == w) {
            synced = true;
         } else {
            if (firsterr < 0)
               firsterr = (int32_t)i;
            errors++;
         }
      } else {
         if (w != exp) {
            if (firsterr < 0)
               firsterr = (int32_t)i;
            errors++;
         }
      }

      // Refill the TX FIFO so the stream keeps flowing.
      if (sent < TOTAL_WORDS) {
         uint32_t txp = 0U;
         while (!sport_tx_ready(SPORT_ID_4, SPORT_HALF_A)) {
            if (++txp > WAIT_TIMEOUT) {
               tx_timeouts++;
               break;
            }
         }
         if (tx_timeouts != 0U)
            break;
         sport_write_raw(SPORT_ID_4, SPORT_HALF_A,
                         lfsr_next(&tx_state));
         sent++;
      }
   }

   // Drain in-flight TX words.
   for (volatile uint32_t k = 0U; k < 1000000U; k++) {
   }
   accum_elapsed_ticks(&elapsed_ticks, &last_tick);

   sport_disable(SPORT_ID_4, SPORT_HALF_A);
   sport_disable(SPORT_ID_4, SPORT_HALF_B);

   bool sport_error_a = sport_has_error(SPORT_ID_4, SPORT_HALF_A);
   bool sport_error_b = sport_has_error(SPORT_ID_4, SPORT_HALF_B);
   uint32_t sport_err_a = MMR(SPORT4A_ERR_REG);
   uint32_t sport_err_b = MMR(SPORT4B_ERR_REG);
   uint64_t bytes = (uint64_t)got * 4ULL;
   uint64_t payload_bits = bytes * BITS_PER_BYTE_64;
   uint64_t rate_bps = 0ULL;
   if (elapsed_ticks != 0ULL) {
      rate_bps =
          udiv64(payload_bits * (uint64_t)SPORT_SCLK_HZ,
                 elapsed_ticks);
   }

   bool pass = (got == TOTAL_WORDS && errors == 0U && firsterr == -1 &&
                rx_timeouts == 0U && tx_timeouts == 0U &&
                !sport_error_a && !sport_error_b && synced &&
                elapsed_ticks >= payload_bits &&
                rate_bps >= MIN_RATE_BPS &&
                rate_bps <= (uint64_t)SPORT_BIT_CLK_HZ);

   put_str("sport_fpga_lb bytes=");
   put_u64(bytes);
   put_str(" words=");
   put_u32(got);
   put_str(" errors=");
   put_u32(errors);
   put_str(" firsterr=");
   put_i32(firsterr);
   put_str(" skipped=");
   put_u32(skipped);
   put_str(" rx_timeouts=");
   put_u32(rx_timeouts);
   put_str(" tx_timeouts=");
   put_u32(tx_timeouts);
   put_str(" sport_error_a=");
   put_u32(sport_error_a ? 1U : 0U);
   put_str(" sport_error_b=");
   put_u32(sport_error_b ? 1U : 0U);
   put_str(" sport_err_a=0x");
   static const char hex[] = "0123456789abcdef";
   for (int i = 7; i >= 0; i--)
      uart_putc(hex[(sport_err_a >> ((unsigned)i * 4U)) & 0xFU]);
   put_str(" sport_err_b=0x");
   for (int i = 7; i >= 0; i--)
      uart_putc(hex[(sport_err_b >> ((unsigned)i * 4U)) & 0xFU]);
   put_str(" bit_clk=");
   put_u32(SPORT_BIT_CLK_HZ);
   put_str(" ticks=");
   put_u64(elapsed_ticks);
   put_str(" rate_bps=");
   put_u64(rate_bps);
   put_str(" ");
   put_str(pass ? "PASS\r\n" : "FAIL\r\n");

   for (;;) {
   }
}
