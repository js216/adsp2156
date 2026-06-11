// SPDX-License-Identifier: MIT
// main.c --- FOUR SPORTs in parallel, each a long
// PRBS31 DMA receiver. The DSP masters each SPORT clock/frame-sync and
// the FPGA streams lockstep PRBS31 data from those clocks.
// Copyright (c) 2026 Jakob Kastelic

#include "board.h"
#include "clocks.h"
#include "dma.h"
#include "gpio.h"
#include "regs.h"
#include "sport.h"
#include "timer.h"
#include "uart.h"
#include <stdbool.h>
#include <stdint.h>

#ifndef TOTAL_WORDS
#define TOTAL_WORDS 16777216U   // per channel (64 MB); aggregate = NCH x this
#endif
#ifndef HALF_WORDS
#define HALF_WORDS        16384U
#endif
#define NCH               4U
#define SPORT_CLKDIV      2U
#define SPORT_FSDIV       31U
#define PRBS31_SEED       0x7FFFFFFFU
#ifndef RX_SAMPLE_RISING
#define RX_SAMPLE_RISING  0
#endif
#ifndef ACTIVE_MASK
#define ACTIVE_MASK       15U
#endif
#ifndef CHECK_MASK
#define CHECK_MASK        ACTIVE_MASK
#endif
#ifndef RX_SHIFT_LEFT_1
#define RX_SHIFT_LEFT_1   0
#endif
#ifndef RX_SHIFT_MASK
#if RX_SHIFT_LEFT_1
#define RX_SHIFT_MASK     CHECK_MASK
#else
#define RX_SHIFT_MASK     0U
#endif
#endif
#ifndef RX_SHIFT_RIGHT_MASK
#define RX_SHIFT_RIGHT_MASK 0U
#endif
#ifndef PINPOINT_START
#define PINPOINT_START    0U
#endif
#ifndef RX_PRBS_SKIP_WORDS
#define RX_PRBS_SKIP_WORDS 0U
#endif

static const enum sport_id SP_ID[NCH] = {
    SPORT_ID_4, SPORT_ID_0, SPORT_ID_5, SPORT_ID_1};
static const enum dma_channel SP_DMA[NCH] = {
    DMA_CH_SPORT4_B, DMA_CH_SPORT0_B, DMA_CH_SPORT5_B, DMA_CH_SPORT1_B};
static const enum sport_half SP_HALF[NCH] = {
    SPORT_HALF_B, SPORT_HALF_B, SPORT_HALF_B, SPORT_HALF_B};

#pragma section("seg_l2_bss", NO_INIT)
static uint32_t rx_buf[NCH][2][HALF_WORDS];
#pragma section("seg_l2_bss", NO_INIT)
static struct dma_dscl rx_desc[NCH][2];

static uint32_t errors_g[NCH];
static int32_t firsterr_g[NCH];
static uint32_t first_got_g[NCH][4];
static uint32_t first_exp_g[4];
static uint32_t prev_g[NCH];
static bool prev_valid_g[NCH];
static uint32_t prbs_g[NCH];
static uint32_t got_words_g[NCH];
static uint32_t timeouts_g;
static uint32_t overruns_g;
static bool first_seen_g;

static void put_str(const char *s) { while (*s) uart_putc(*s++); }

static void put_u32(uint32_t v)
{
   static const uint32_t p10[10] = {1000000000U,100000000U,10000000U,1000000U,
       100000U,10000U,1000U,100U,10U,1U};
   if (!v) { uart_putc('0'); return; }
   unsigned st = 0U;
   for (unsigned i = 0U; i < 10U; i++) {
      char c = '0';
      while (v >= p10[i]) { v -= p10[i]; c++; }
      if (c != '0') st = 1U;
      if (st) uart_putc(c);
   }
}

static void put_u64(uint64_t v)
{
   static const uint64_t p10[20] = {
       10000000000000000000ULL, 1000000000000000000ULL,
       100000000000000000ULL,   10000000000000000ULL,
       1000000000000000ULL,     100000000000000ULL,
       10000000000000ULL,       1000000000000ULL,
       100000000000ULL,         10000000000ULL,
       1000000000ULL,           100000000ULL,
       10000000ULL,             1000000ULL,
       100000ULL,               10000ULL,
       1000ULL,                 100ULL,
       10ULL,                   1ULL};
   if (!v) { uart_putc('0'); return; }
   unsigned st = 0U;
   for (unsigned i = 0U; i < 20U; i++) {
      char c = '0';
      while (v >= p10[i]) { v -= p10[i]; c++; }
      if (c != '0') st = 1U;
      if (st) uart_putc(c);
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

static void put_hex32(uint32_t v)
{
   static const char hex[] = "0123456789ABCDEF";
   for (int i = 7; i >= 0; i--)
      uart_putc(hex[(v >> ((uint32_t)i * 4U)) & 0xFU]);
}

static inline uint32_t prbs31_word(uint32_t *state)
{
   uint32_t s = *state, w = 0U;
#define PRBS_STEP() do { \
      uint32_t nb = ((s >> 30) ^ (s >> 27)) & 1U; \
      s = ((s << 1) | nb) & 0x7FFFFFFFU; \
      w = (w << 1) | nb; \
   } while (0)
   PRBS_STEP(); PRBS_STEP(); PRBS_STEP(); PRBS_STEP();
   PRBS_STEP(); PRBS_STEP(); PRBS_STEP(); PRBS_STEP();
   PRBS_STEP(); PRBS_STEP(); PRBS_STEP(); PRBS_STEP();
   PRBS_STEP(); PRBS_STEP(); PRBS_STEP(); PRBS_STEP();
   PRBS_STEP(); PRBS_STEP(); PRBS_STEP(); PRBS_STEP();
   PRBS_STEP(); PRBS_STEP(); PRBS_STEP(); PRBS_STEP();
   PRBS_STEP(); PRBS_STEP(); PRBS_STEP(); PRBS_STEP();
   PRBS_STEP(); PRBS_STEP(); PRBS_STEP(); PRBS_STEP();
#undef PRBS_STEP
   *state = s;
   return w;
}

static inline bool active_ch(uint32_t c)
{
   return ((ACTIVE_MASK >> c) & 1U) != 0U;
}

static inline bool check_ch(uint32_t c)
{
   return ((CHECK_MASK >> c) & 1U) != 0U;
}

static inline bool shift_ch(uint32_t c)
{
   return ((RX_SHIFT_MASK >> c) & 1U) != 0U;
}

static inline bool shift_right_ch(uint32_t c)
{
   return ((RX_SHIFT_RIGHT_MASK >> c) & 1U) != 0U;
}

static uint32_t dma_which_half(enum dma_channel ch, const uint32_t *b0,
                               const uint32_t *b1)
{
   uint32_t addr = dma_addr_cur(ch);
   uint32_t a0 = (uint32_t)b0, a1 = (uint32_t)b1, bytes = HALF_WORDS * 4U;
   if (addr >= a1 && addr < a1 + bytes) return 1U;
   if (addr >= a0 && addr < a0 + bytes) return 0U;
   return 2U;
}

static bool dma_still_on_half(enum dma_channel ch, const uint32_t *b0,
                              const uint32_t *b1, uint32_t completed)
{
   for (uint32_t i = 0U; i < 10000U; i++) {
      if (dma_which_half(ch, b0, b1) != completed)
         return false;
   }
   return dma_which_half(ch, b0, b1) == completed;
}

static void clear_first_seen(void)
{
   if (first_seen_g)
      return;
   for (uint32_t cc = 0U; cc < NCH; cc++)
      for (uint32_t j = 0U; j < 4U; j++)
         first_got_g[cc][j] = 0U;
   first_seen_g = true;
}

#ifdef DIAG_BOUNDARY
// Raw got/expected words around index DIAG_BOUNDARY for lane DIAG_BND_LANE.
static uint32_t bnd_got_g[8];
static uint32_t bnd_exp_g[8];
#ifndef DIAG_BND_LANE
#define DIAG_BND_LANE 1U
#endif
#endif

static void check_word(uint32_t c, uint32_t word, uint32_t expected,
                       uint32_t idx)
{
#ifdef DIAG_BOUNDARY
   if (c == DIAG_BND_LANE && idx >= (DIAG_BOUNDARY) - 4U &&
       idx < (DIAG_BOUNDARY) + 4U) {
      bnd_got_g[idx - ((DIAG_BOUNDARY) - 4U)] = word;
      bnd_exp_g[idx - ((DIAG_BOUNDARY) - 4U)] = expected;
   }
#endif
   if (word != expected) {
      if (firsterr_g[c] < 0)
         firsterr_g[c] = (int32_t)idx;
      if (errors_g[c] != UINT32_MAX)
         errors_g[c]++;
   }
}

static void check_half(uint32_t c, uint32_t completed)
{
   bool shifted = shift_ch(c);
   bool shifted_right = shift_right_ch(c);
   for (uint32_t i = 0U; i < HALF_WORDS; i++) {
      bool had_prev = prev_valid_g[c];
      uint32_t expected = 0U;
      uint32_t checked = got_words_g[c] - RX_PRBS_SKIP_WORDS;
      bool in_prbs = got_words_g[c] >= RX_PRBS_SKIP_WORDS;
      if (in_prbs && (!shifted || had_prev) && checked < TOTAL_WORDS)
         expected = prbs31_word(&prbs_g[c]);
      uint32_t word = rx_buf[c][completed][i];
      if (shifted_right && in_prbs && checked < TOTAL_WORDS) {
         uint32_t aligned = (word >> 1U) | (had_prev ? (prev_g[c] << 31U) : 0U);
         clear_first_seen();
         if (checked < 4U) {
            first_exp_g[checked] = expected;
            first_got_g[c][checked] = aligned;
         }
         check_word(c, aligned, expected, checked);
         prev_g[c] = word;
         prev_valid_g[c] = true;
      } else if (shifted) {
         if (had_prev && in_prbs && checked < TOTAL_WORDS) {
            uint32_t aligned = (prev_g[c] << 1U) | (word >> 31U);
            clear_first_seen();
            if (checked < 4U) {
               first_exp_g[checked] = expected;
               first_got_g[c][checked] = aligned;
            }
            check_word(c, aligned, expected, checked);
         }
         prev_g[c] = word;
         prev_valid_g[c] = true;
      } else if (in_prbs && checked < TOTAL_WORDS) {
         clear_first_seen();
         if (checked < 4U) {
            first_exp_g[checked] = expected;
            first_got_g[c][checked] = word;
         }
         check_word(c, word, expected, checked);
      }
      if ((!shifted || had_prev || shifted_right) &&
          got_words_g[c] < TOTAL_WORDS + RX_PRBS_SKIP_WORDS)
         got_words_g[c]++;
   }
}

// DSP masters each lane's clock/frame-sync (internal clocking); the FPGA
// launches data against the forwarded pin clock. Mirrors sport_fpga_2x's
// working receiver configuration.
static struct sport_dsp_cfg rx_slave_cfg = {
    .word_bits = 32, .clkdiv = SPORT_CLKDIV, .fsdiv = SPORT_FSDIV, .is_tx = false,
    .internal_clk = true, .internal_fs = true, .late_fs = false,
    .data_indep_fs = false, .sample_rising = RX_SAMPLE_RISING != 0,
};

int main(void)
{
   static const struct clocks_cfg clk = BOARD_CLOCKS_CFG;
   clocks_init(&clk);
   uart_init(BOARD_BAUD_DIV);
   timer_init();
   board_som_init(0U);
   gpio_make_output(GPIO_DAI1_06);
   gpio_write(GPIO_DAI1_06, false);

   put_str("\r\nsport_4x boot prbs31 seed=0x7FFFFFFF channels=4\r\n");

   for (uint32_t c = PINPOINT_START; c < NCH; c++) {
      if (SP_ID[c] == SPORT_ID_4)
         sport_route_rx_master_to_pins(SP_ID[c], 1U, 5U, 7U, 8U);
      else if (SP_ID[c] == SPORT_ID_0)
         sport_route_rx_master_to_pins(SP_ID[c], 0U, 5U, 7U, 8U);
      else if (SP_ID[c] == SPORT_ID_5)
         sport_route_rx_master_to_pins(SP_ID[c], 1U, 9U, 10U, 11U);
      else
         sport_route_rx_master_to_pins(SP_ID[c], 0U, 10U, 12U, 20U);
      if (!active_ch(c))
         continue;
      sport_dsp_serial_init(SP_ID[c], SP_HALF[c], &rx_slave_cfg);
      sport_clear_errors(SP_ID[c], SP_HALF[c]);
   }

   for (uint32_t c = 0U; c < NCH; c++) {
      errors_g[c] = 0U;
      firsterr_g[c] = -1;
      prev_g[c] = 0U;
      prev_valid_g[c] = false;
      prbs_g[c] = PRBS31_SEED;
      got_words_g[c] = 0U;
      for (uint32_t j = 0U; j < 4U; j++)
         first_got_g[c][j] = 0U;
   }
   for (uint32_t j = 0U; j < 4U; j++)
      first_exp_g[j] = 0U;
   timeouts_g = 0U;
   overruns_g = 0U;
   first_seen_g = false;
   uint64_t elapsed = 0ULL;

   for (uint32_t c = PINPOINT_START; c < NCH; c++)
      if (active_ch(c))
         dma_pingpong_rx_config(SP_DMA[c], rx_buf[c][0], rx_buf[c][1],
                                HALF_WORDS, rx_desc[c]);
   put_str("DMACFG\r\n");
   gpio_write(GPIO_DAI1_06, true);
   put_str("RUN\r\n");
   for (uint32_t c = PINPOINT_START; c < NCH; c++)
      if (active_ch(c))
         sport_enable(SP_ID[c], SP_HALF[c]);

   {
      uint32_t last_tick = timer_ticks();

      uint32_t n_halves[NCH];
      uint32_t done_halves[NCH] = {0U, 0U, 0U, 0U};
      for (uint32_t c = 0U; c < NCH; c++) {
         uint32_t capture_words = TOTAL_WORDS + RX_PRBS_SKIP_WORDS +
                                  (shift_ch(c) ? 1U : 0U);
         n_halves[c] = (capture_words + HALF_WORDS - 1U) / HALF_WORDS;
      }

      while (timeouts_g == 0U && overruns_g == 0U) {
         bool all_done = true;
         for (uint32_t c = 0U; c < NCH; c++) {
            if (check_ch(c) && done_halves[c] < n_halves[c])
               all_done = false;
         }
         if (all_done)
            break;

         uint32_t deadline = last_tick + (BOARD_SCLK_HZ * 2U);
         bool got_one = false;
         while (!got_one) {
            for (uint32_t c = 0U; c < NCH; c++) {
               if (!check_ch(c) || done_halves[c] >= n_halves[c])
                  continue;
               if (dma_wrap_check(SP_DMA[c])) {
                  uint32_t current = dma_which_half(SP_DMA[c], rx_buf[c][0],
                                                    rx_buf[c][1]);
                  if (current > 1U) {
                     overruns_g++;
                     got_one = true;
                     break;
                  }
                  uint32_t completed = current ^ 1U;
                  if (dma_still_on_half(SP_DMA[c], rx_buf[c][0],
                                        rx_buf[c][1], completed)) {
                     overruns_g++;
                     got_one = true;
                     break;
                  }
                  check_half(c, completed);
                  done_halves[c]++;
                  got_one = true;
               }
            }
            uint32_t now = timer_ticks();
            elapsed += (uint64_t)(uint32_t)(now - last_tick);
            last_tick = now;
            if ((int32_t)(last_tick - deadline) >= 0) {
               timeouts_g++;
               break;
            }
         }
      }

      for (uint32_t c = 0U; c < NCH; c++) {
         if (active_ch(c)) {
            dma_disable(SP_DMA[c]);
            dma_wait_idle(SP_DMA[c]);
         }
      }
   }

   for (uint32_t c = 0U; c < NCH; c++)
      if (active_ch(c))
         sport_disable(SP_ID[c], SP_HALF[c]);

   uint32_t total_err = 0U;
   for (uint32_t c = 0U; c < NCH; c++) {
      if (check_ch(c))
         total_err += errors_g[c];
   }
   uint64_t per_ch_bytes = UINT64_MAX;
   uint32_t checked = 0U;
   for (uint32_t c = 0U; c < NCH; c++) {
      if (check_ch(c)) {
         checked++;
         uint32_t checked_words = got_words_g[c] > RX_PRBS_SKIP_WORDS
                                      ? got_words_g[c] - RX_PRBS_SKIP_WORDS
                                      : 0U;
         uint64_t ch_bytes = (uint64_t)checked_words * 4ULL;
         if (ch_bytes < per_ch_bytes)
            per_ch_bytes = ch_bytes;
      }
   }
   if (per_ch_bytes == UINT64_MAX)
      per_ch_bytes = 0ULL;
   uint64_t agg_bytes = per_ch_bytes * (uint64_t)checked;
   bool pass = (total_err == 0U && timeouts_g == 0U && overruns_g == 0U);
   for (uint32_t c = 0U; c < NCH; c++) {
      if (check_ch(c) && got_words_g[c] < TOTAL_WORDS + RX_PRBS_SKIP_WORDS)
         pass = false;
   }

#ifdef DIAG_BOUNDARY
   put_str("bnd got/exp @");
   put_u32((DIAG_BOUNDARY) - 4U);
   put_str(":");
   for (uint32_t i = 0U; i < 8U; i++) {
      put_str(" ");
      put_hex32(bnd_got_g[i]);
      put_str("/");
      put_hex32(bnd_exp_g[i]);
   }
   put_str("\r\n");
#endif
   put_str("sport_4x agg_bytes="); put_u64(agg_bytes);
   put_str(" per_ch_bytes="); put_u64(per_ch_bytes);
   put_str(" errors0="); put_u32(errors_g[0]);
   put_str(" errors1="); put_u32(errors_g[1]);
   put_str(" errors2="); put_u32(errors_g[2]);
   put_str(" errors3="); put_u32(errors_g[3]);
   put_str(" firsterr0="); put_i32(firsterr_g[0]);
   put_str(" firsterr1="); put_i32(firsterr_g[1]);
   put_str(" firsterr2="); put_i32(firsterr_g[2]);
   put_str(" firsterr3="); put_i32(firsterr_g[3]);
   for (uint32_t c = 0U; c < NCH; c++) {
      put_str(" first");
      put_u32(c);
      put_str("=");
      for (uint32_t i = 0U; i < 4U; i++) {
         if (i) uart_putc(',');
         put_hex32(first_got_g[c][i]);
      }
      put_str("/");
      for (uint32_t i = 0U; i < 4U; i++) {
         if (i) uart_putc(',');
         put_hex32(first_exp_g[i]);
      }
   }
   put_str(" timeouts="); put_u32(timeouts_g);
   put_str(" overruns="); put_u32(overruns_g);
   put_str(pass ? " PASS\r\n" : " FAIL\r\n");

   for (;;) { }
}
