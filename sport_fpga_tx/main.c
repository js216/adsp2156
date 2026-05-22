// SPDX-License-Identifier: MIT
// main.c --- SPORT4 half-B long PRBS DMA receiver
// Copyright (c) 2026 Jakob Kastelic

#include "board.h"
#include "clocks.h"
#include "dma.h"
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
#define HALF_WORDS        1024U
#define BIT_CLK_HZ        62250000U
#define MAX_RATE_BPS      62500000U
#define MIN_RATE_BPS      49800000U
#define START_TIMEOUT     1000U
#define START_WAIT_LOOPS  2000000U
#define PRBS31_SEED       0x7FFFFFFFU
#define SCLK_HZ_64        93750000ULL
#define BITS_PER_BYTE_64  8ULL
#define SPORT4B_ERR       0x310024A0U

#pragma section("seg_l2_bss", NO_INIT)
static uint32_t rx_a[HALF_WORDS];
#pragma section("seg_l2_bss", NO_INIT)
static uint32_t rx_pong[HALF_WORDS];
#pragma section("seg_l2_bss", NO_INIT)
static struct dma_dscl rx_desc[2];

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
       10000000000000000000ULL,
       1000000000000000000ULL,
       100000000000000000ULL,
       10000000000000000ULL,
       1000000000000000ULL,
       100000000000000ULL,
       10000000000000ULL,
       1000000000000ULL,
       100000000000ULL,
       10000000000ULL,
       1000000000ULL,
       100000000ULL,
       10000000ULL,
       1000000ULL,
       100000ULL,
       10000ULL,
       1000ULL,
       100ULL,
       10ULL,
       1ULL,
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

// PRBS-31, polynomial x^31 + x^28 + 1, seed 0x7fffffff.
// Generates serial bits with new_bit = state[30] ^ state[27] and packs
// 32 output bits per SPORT word, first generated bit into word bit 31.
static inline uint32_t prbs31_word(uint32_t *state)
{
   uint32_t s = *state & 0x7FFFFFFFU;
   uint32_t word = 0U;
   for (uint32_t i = 0U; i < 32U; i++) {
      uint32_t new_bit = ((s >> 30U) ^ (s >> 27U)) & 1U;
      s = ((s << 1U) | new_bit) & 0x7FFFFFFFU;
      word = (word << 1U) | new_bit;
   }
   *state = s;
   return word;
}

static inline void accum_elapsed_ticks(uint64_t *elapsed_ticks64,
                                       uint32_t *last_tick)
{
   uint32_t now = timer_ticks();
   *elapsed_ticks64 += (uint64_t)(uint32_t)(now - *last_tick);
   *last_tick = now;
}

static bool discard_until_stream_gap(void)
{
   uint32_t scratch = 0U;
   bool saw_activity = false;

   while (sport_read(SPORT_ID_4, SPORT_HALF_B, &scratch, START_TIMEOUT) == 0) {
   }
   for (uint32_t i = 0U; i < START_WAIT_LOOPS; i++) {
      if (sport_read(SPORT_ID_4, SPORT_HALF_B, &scratch, START_TIMEOUT) == 0) {
         saw_activity = true;
      } else if (saw_activity) {
         return true;
      }
   }
   return false;
}

static uint32_t dma_current_half(void)
{
   uint32_t addr = dma_addr_cur(DMA_CH_SPORT4_B);
   uint32_t b0 = (uint32_t)rx_a;
   uint32_t b1 = (uint32_t)rx_pong;
   uint32_t bytes = HALF_WORDS * 4U;

   if (addr >= b1 && addr < (b1 + bytes))
      return 1U;
   if (addr >= b0 && addr < (b0 + bytes))
      return 0U;
   return 2U;
}

static void check_half(const uint32_t *buf, uint32_t base_index,
                       uint32_t *errors, int32_t *firsterr)
{
   static uint32_t prbs_state = PRBS31_SEED;
   (void)base_index;
   for (uint32_t i = 0U; i < HALF_WORDS; i++) {
      uint32_t expected = prbs31_word(&prbs_state);
      if (buf[i] != expected) {
         if (*firsterr < 0)
            *firsterr = (int32_t)(base_index + i);
         if (*errors != UINT32_MAX)
            (*errors)++;
      }
   }
}

static struct sport_dsp_cfg rx_slave_cfg = {
    .word_bits     = 32,
    .clkdiv        = 0,
    .fsdiv         = 0,
    .is_tx         = false,
    .internal_clk  = false,
    .internal_fs   = false,
    .late_fs       = true,
    .data_indep_fs = false,
    .sample_rising = true,
};

int main(void)
{
   static const struct clocks_cfg clk = BOARD_CLOCKS_CFG;
   clocks_init(&clk);
   uart_init(BOARD_BAUD_DIV);
   timer_init();
   board_som_init(0U);

   put_str("\r\nsport_fpga_tx_prbs_long boot prbs31 poly=x^31+x^28+1 seed=0x7FFFFFFF pack=msb_first_output_bits\r\n");

   sport_enable_external_pins(SPORT_ID_4);
   sport_install_external_loopback(SPORT_ID_4);
   sport_dsp_serial_init(SPORT_ID_4, SPORT_HALF_B, &rx_slave_cfg);
   sport_clear_errors(SPORT_ID_4, SPORT_HALF_B);
   sport_enable(SPORT_ID_4, SPORT_HALF_B);

   uint32_t errors = 0U;
   uint32_t timeouts = 0U;
   uint32_t overruns = 0U;
   uint32_t wrap_misses = 0U;
   uint32_t got_words = 0U;
   int32_t firsterr = -1;
   uint64_t elapsed_ticks64 = 0ULL;

   if (!discard_until_stream_gap()) {
      timeouts++;
   } else {
      dma_pingpong_rx_config(DMA_CH_SPORT4_B, rx_a, rx_pong, HALF_WORDS,
                             rx_desc);
      uint32_t last_tick = timer_ticks();

      for (uint32_t half = 0U; half < (TOTAL_WORDS / HALF_WORDS); half++) {
         uint32_t deadline = last_tick + (BOARD_SCLK_HZ * 2U);
         while (!dma_wrap_check(DMA_CH_SPORT4_B)) {
            accum_elapsed_ticks(&elapsed_ticks64, &last_tick);
            if ((int32_t)(last_tick - deadline) >= 0) {
               timeouts++;
               break;
            }
         }
         if (timeouts != 0U)
            break;

         uint32_t completed_half = half & 1U;
         uint32_t current_half = dma_current_half();
         if (current_half == completed_half) {
            overruns++;
            break;
         }
         if (current_half > 1U)
            wrap_misses++;

         check_half(completed_half == 0U ? rx_a : rx_pong, got_words, &errors,
                    &firsterr);
         got_words += HALF_WORDS;
         accum_elapsed_ticks(&elapsed_ticks64, &last_tick);
      }

      dma_disable(DMA_CH_SPORT4_B);
      dma_wait_idle(DMA_CH_SPORT4_B);
   }

   sport_disable(SPORT_ID_4, SPORT_HALF_B);
   bool sport_error = sport_has_error(SPORT_ID_4, SPORT_HALF_B);
   uint32_t sport_err = MMR(SPORT4B_ERR);
   uint64_t bytes = (uint64_t)got_words * 4ULL;
   uint64_t rate_bps = 0ULL;
   if (elapsed_ticks64 != 0ULL) {
      rate_bps = udiv64((uint64_t)bytes * BITS_PER_BYTE_64 * SCLK_HZ_64,
                        elapsed_ticks64);
   }

   bool pass = (bytes == TOTAL_BYTES && got_words == TOTAL_WORDS &&
                errors == 0U && firsterr == -1 && timeouts == 0U &&
                overruns == 0U && wrap_misses == 0U && !sport_error &&
                BIT_CLK_HZ >= 62000000U && BIT_CLK_HZ <= 62500000U &&
                rate_bps >= MIN_RATE_BPS && rate_bps <= MAX_RATE_BPS);

   put_str("sport_fpga_tx_prbs_long bytes=");
   put_u64(bytes);
   put_str(" words=");
   put_u32(got_words);
   put_str(" errors=");
   put_u32(errors);
   put_str(" firsterr=");
   put_i32(firsterr);
   put_str(" timeouts=");
   put_u32(timeouts);
   put_str(" overruns=");
   put_u32(overruns);
   put_str(" wrap_misses=");
   put_u32(wrap_misses);
   put_str(" sport_error=");
   put_u32(sport_error ? 1U : 0U);
   put_str(" sport_err=0x");
   static const char hex[] = "0123456789abcdef";
   for (int i = 7; i >= 0; i--)
      uart_putc(hex[(sport_err >> ((unsigned)i * 4U)) & 0xFU]);
   put_str(" bit_clk_hz=");
   put_u32(BIT_CLK_HZ);
   put_str(" ticks=");
   put_u64(elapsed_ticks64);
   put_str(" rate_bps=");
   put_u64(rate_bps);
   put_str(" ");
   put_str(pass ? "PASS\r\n" : "FAIL\r\n");

   for (;;) {
   }
}
