// SPDX-License-Identifier: MIT
// main.c --- TWO SPORTs in parallel (SPORT4B + SPORT0B), each a long
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
#define HALF_WORDS        32768U
#endif
#define PRBS31_SEED       0x7FFFFFFFU
#define NCH               2U
#ifndef SPORT_CLKDIV
#define SPORT_CLKDIV      2U
#endif
#ifndef SPORT_SCLK_HZ
#define SPORT_SCLK_HZ     BOARD_SCLK_HZ
#endif
#define SPORT_FSDIV       31U
#ifndef PINPOINT_START
#define PINPOINT_START    0U
#endif
#ifndef ACTIVE_MASK
#define ACTIVE_MASK       3U
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
#ifndef RX_LATE_FS
#define RX_LATE_FS        0
#endif
#ifndef RX_SAMPLE_RISING
#define RX_SAMPLE_RISING  1
#endif
#ifndef LEAVE_RUNNING_AFTER_REPORT
#define LEAVE_RUNNING_AFTER_REPORT 0
#endif

#ifdef USE_SPORT5_0
static const enum sport_id    SP_ID[NCH]  = { SPORT_ID_5, SPORT_ID_0 };
static const enum dma_channel SP_DMA[NCH] = { DMA_CH_SPORT5_B, DMA_CH_SPORT0_B };
static const enum sport_half  SP_HALF[NCH] = { SPORT_HALF_B, SPORT_HALF_B };
#elif defined(USE_SPORT0_5)
static const enum sport_id    SP_ID[NCH]  = { SPORT_ID_0, SPORT_ID_5 };
static const enum dma_channel SP_DMA[NCH] = { DMA_CH_SPORT0_B, DMA_CH_SPORT5_B };
static const enum sport_half  SP_HALF[NCH] = { SPORT_HALF_B, SPORT_HALF_B };
#elif defined(USE_SPORT0_1)
static const enum sport_id    SP_ID[NCH]  = { SPORT_ID_0, SPORT_ID_1 };
static const enum dma_channel SP_DMA[NCH] = { DMA_CH_SPORT0_B, DMA_CH_SPORT1_B };
static const enum sport_half  SP_HALF[NCH] = { SPORT_HALF_B, SPORT_HALF_B };
#elif defined(USE_SPORT4A_0)
static const enum sport_id    SP_ID[NCH]  = { SPORT_ID_4, SPORT_ID_0 };
static const enum dma_channel SP_DMA[NCH] = { DMA_CH_SPORT4_A, DMA_CH_SPORT0_B };
static const enum sport_half  SP_HALF[NCH] = { SPORT_HALF_A, SPORT_HALF_B };
#else
static const enum sport_id    SP_ID[NCH]  = { SPORT_ID_4, SPORT_ID_0 };
static const enum dma_channel SP_DMA[NCH] = { DMA_CH_SPORT4_B, DMA_CH_SPORT0_B };
static const enum sport_half  SP_HALF[NCH] = { SPORT_HALF_B, SPORT_HALF_B };
#endif

#define SPORT4B_CTL 0x31002480U
#define SPORT4B_ERR 0x310024A0U
#define SPORT5B_CTL 0x31002580U
#define SPORT5B_ERR 0x310025A0U
#define SPORT0B_CTL 0x31002080U
#define SPORT0B_ERR 0x310020A0U
#define SPORT1B_CTL 0x31002180U
#define SPORT1B_ERR 0x310021A0U

#ifdef USE_SPORT5_0
static const uint32_t SP_CTL_REG[NCH] = { SPORT5B_CTL, SPORT0B_CTL };
static const uint32_t SP_ERR_REG[NCH] = { SPORT5B_ERR, SPORT0B_ERR };
#elif defined(USE_SPORT0_1)
static const uint32_t SP_CTL_REG[NCH] = { SPORT0B_CTL, SPORT1B_CTL };
static const uint32_t SP_ERR_REG[NCH] = { SPORT0B_ERR, SPORT1B_ERR };
#else
static const uint32_t SP_CTL_REG[NCH] = { SPORT4B_CTL, SPORT0B_CTL };
static const uint32_t SP_ERR_REG[NCH] = { SPORT4B_ERR, SPORT0B_ERR };
#endif

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
static uint64_t elapsed_g;
static bool first_seen_g;
static uint32_t dma_stat_before_g[NCH];
static uint32_t sport_ctl_before_g[NCH];
static uint32_t sport_err_before_g[NCH];
static uint32_t sport_err_after_g[NCH];

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

static void put_i32(int32_t v)
{
   if (v < 0) {
      uart_putc('-');
      put_u32((uint32_t)(-v));
   } else {
      put_u32((uint32_t)v);
   }
}

static void put_u64(uint64_t v)
{
   uint32_t billions = 0U;
   const uint64_t one_billion = 1000000000ULL;
   while (v >= one_billion) {
      v -= one_billion;
      billions++;
   }
   if (billions == 0U) {
      put_u32((uint32_t)v);
      return;
   }
   put_u32(billions);
   uint32_t rem = (uint32_t)v;
   uint32_t div = 100000000U;
   while (div != 0U) {
      uint32_t digit = 0U;
      while (rem >= div) {
         rem -= div;
         digit++;
      }
      uart_putc((char)('0' + digit));
      div /= 10U;
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

static uint32_t dma_which_half(enum dma_channel ch, const uint32_t *b0,
                               const uint32_t *b1)
{
   uint32_t addr = dma_addr_cur(ch);
   uint32_t a0 = (uint32_t)b0, a1 = (uint32_t)b1, bytes = HALF_WORDS * 4U;
   if (addr >= a1 && addr < a1 + bytes) return 1U;
   if (addr >= a0 && addr < a0 + bytes) return 0U;
   return 2U;
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

static uint32_t active_count(void)
{
   uint32_t n = 0U;
   for (uint32_t c = 0U; c < NCH; c++) {
      if (active_ch(c))
         n++;
   }
   return n;
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

static void check_word(uint32_t c, uint32_t word, uint32_t expected,
                       uint32_t errors[NCH], int32_t firsterr[NCH],
                       uint32_t idx)
{
   if (word != expected) {
      if (firsterr[c] < 0)
         firsterr[c] = (int32_t)idx;
      if (errors[c] != UINT32_MAX)
         errors[c]++;
   }
}

static void check_half(uint32_t c, uint32_t completed)
{
   bool shifted = shift_ch(c);
   bool shifted_right = shift_right_ch(c);
   for (uint32_t i = 0U; i < HALF_WORDS; i++) {
      bool had_prev = prev_valid_g[c];
      uint32_t expected = 0U;
      if ((!shifted || had_prev) && got_words_g[c] < TOTAL_WORDS)
         expected = prbs31_word(&prbs_g[c]);
      uint32_t word = rx_buf[c][completed][i];
      if (shifted_right && got_words_g[c] < TOTAL_WORDS) {
         uint32_t aligned = (word >> 1U) | (had_prev ? (prev_g[c] << 31U) : 0U);
         if (!first_seen_g) {
            for (uint32_t cc = 0U; cc < NCH; cc++)
               for (uint32_t j = 0U; j < 4U; j++)
                  first_got_g[cc][j] = 0U;
            first_seen_g = true;
         }
         if (got_words_g[c] < 4U) {
            first_exp_g[got_words_g[c]] = expected;
            first_got_g[c][got_words_g[c]] = aligned;
         }
         check_word(c, aligned, expected, errors_g, firsterr_g, got_words_g[c]);
         prev_g[c] = word;
         prev_valid_g[c] = true;
      } else if (shifted) {
         if (had_prev && got_words_g[c] < TOTAL_WORDS) {
         uint32_t aligned = (prev_g[c] << 1U) | (word >> 31U);
         if (!first_seen_g) {
            for (uint32_t cc = 0U; cc < NCH; cc++)
               for (uint32_t j = 0U; j < 4U; j++)
                  first_got_g[cc][j] = 0U;
            first_seen_g = true;
         }
         if (got_words_g[c] < 4U) {
            first_exp_g[got_words_g[c]] = expected;
         first_got_g[c][got_words_g[c]] = aligned;
         }
         check_word(c, aligned, expected, errors_g, firsterr_g, got_words_g[c]);
         }
         prev_g[c] = word;
         prev_valid_g[c] = true;
      } else if (got_words_g[c] < TOTAL_WORDS) {
         if (!first_seen_g) {
            for (uint32_t cc = 0U; cc < NCH; cc++)
               for (uint32_t j = 0U; j < 4U; j++)
                  first_got_g[cc][j] = rx_buf[cc][completed][j];
            uint32_t p = PRBS31_SEED;
            for (uint32_t j = 0U; j < 4U; j++)
               first_exp_g[j] = prbs31_word(&p);
            first_seen_g = true;
         }
         check_word(c, word, expected, errors_g, firsterr_g, got_words_g[c]);
      }
      if ((!shifted || had_prev || shifted_right) && got_words_g[c] < TOTAL_WORDS)
         got_words_g[c]++;
   }
}

static struct sport_dsp_cfg rx_slave_cfg = {
    .word_bits = 32, .clkdiv = SPORT_CLKDIV, .fsdiv = SPORT_FSDIV, .is_tx = false,
    .internal_clk = true, .internal_fs = true, .late_fs = RX_LATE_FS != 0,
    .data_indep_fs = false, .sample_rising = RX_SAMPLE_RISING != 0,
};

int main(void)
{
   static const struct clocks_cfg clk =
#if SPORT_SCLK_HZ == 92187500U
      {
         .clkin_hz = 25000000U,
         .msel     = 59U,
         .df       = 0U,
         .csel     = 2U,
         .syssel   = 4U,
         .s0sel    = 4U,
         .s1sel    = 2U,
         .dsel     = 3U,
         .osel     = 40U,
      };
#elif SPORT_SCLK_HZ == 59375000U
      CLOCKS_CFG_SCLK0_59MHZ;
#else
      BOARD_CLOCKS_CFG;
#endif
   clocks_init(&clk);
   uart_init(BOARD_BAUD_DIV);
   timer_init();
   board_som_init(0U);
   gpio_make_output(GPIO_DAI1_06);
   gpio_write(GPIO_DAI1_06, false);

   put_str("\r\nsport_2x boot prbs31 seed=0x7FFFFFFF active_mask=0x");
   uart_putc((char)('0' + (ACTIVE_MASK & 0xFU)));
   put_str(" active_channels=");
   put_u32(active_count());
#ifdef USE_SPORT5_0
   put_str(" (SPORT5B+SPORT0B)\r\n");
#elif defined(USE_SPORT0_1)
   put_str(" (SPORT0B+SPORT1B)\r\n");
#else
   put_str(" (SPORT4B+SPORT0B)\r\n");
#endif

   for (uint32_t c = PINPOINT_START; c < NCH; c++) {
      if (!active_ch(c))
         continue;
      put_str("ROUTE");
      put_u32(c);
      put_str("\r\n");
      if (SP_ID[c] == SPORT_ID_4 && SP_HALF[c] == SPORT_HALF_A)
         sport_route_rx_master_a_to_pins(SP_ID[c], 1U, 5U, 7U, 8U);
      else if (SP_ID[c] == SPORT_ID_4)
         sport_route_rx_master_to_pins(SP_ID[c], 1U, 5U, 7U, 8U);
      else if (SP_ID[c] == SPORT_ID_0)
         sport_route_rx_master_to_pins(SP_ID[c], 0U, 5U, 7U, 8U);
      else if (SP_ID[c] == SPORT_ID_5)
         sport_route_rx_master_to_pins(SP_ID[c], 1U, 9U, 10U, 11U);
      else
         sport_route_rx_master_to_pins(SP_ID[c], 0U, 10U, 12U, 20U);
      put_str("INIT");
      put_u32(c);
      put_str("\r\n");
      sport_dsp_serial_init(SP_ID[c], SP_HALF[c], &rx_slave_cfg);
      put_str("CLEAR");
      put_u32(c);
      put_str("\r\n");
      sport_clear_errors(SP_ID[c], SP_HALF[c]);
   }

   for (uint32_t c = 0U; c < NCH; c++) {
      errors_g[c] = 0U;
      firsterr_g[c] = -1;
      prev_g[c] = 0U;
      prev_valid_g[c] = false;
      dma_stat_before_g[c] = 0U;
      sport_ctl_before_g[c] = 0U;
      sport_err_before_g[c] = 0U;
      sport_err_after_g[c] = 0U;
      for (uint32_t j = 0U; j < 4U; j++)
         first_got_g[c][j] = 0U;
   }
   for (uint32_t j = 0U; j < 4U; j++)
      first_exp_g[j] = 0U;
   for (uint32_t c = 0U; c < NCH; c++) {
      prbs_g[c] = PRBS31_SEED;
      got_words_g[c] = 0U;
   }
   timeouts_g = 0U;
   overruns_g = 0U;
   elapsed_g = 0ULL;
   first_seen_g = false;

   for (uint32_t c = PINPOINT_START; c < NCH; c++) {
      if (!active_ch(c))
         continue;
      put_str("DMA");
      put_u32(c);
      put_str("\r\n");
      dma_pingpong_rx_config(SP_DMA[c], rx_buf[c][0], rx_buf[c][1],
                             HALF_WORDS, rx_desc[c]);
      put_str("DMAOK");
      put_u32(c);
      put_str("\r\n");
   }
   put_str("DMACFG\r\n");
   gpio_write(GPIO_DAI1_06, true);
   put_str("RUN\r\n");
   delay_us(10U);
   for (uint32_t c = PINPOINT_START; c < NCH; c++) {
      if (!active_ch(c))
         continue;
      sport_enable(SP_ID[c], SP_HALF[c]);
   }

   {
      uint32_t last_tick = timer_ticks();

      uint32_t n_halves[NCH];
      for (uint32_t c = 0U; c < NCH; c++) {
         uint32_t capture_words = TOTAL_WORDS + (shift_ch(c) ? 1U : 0U);
         n_halves[c] = (capture_words + HALF_WORDS - 1U) / HALF_WORDS;
      }
      uint32_t done_halves[NCH] = {0U, 0U};
      while (timeouts_g == 0U) {
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
                  check_half(c, completed);
                  done_halves[c]++;
                  got_one = true;
               }
            }
            if (overruns_g)
               break;
            uint32_t now = timer_ticks();
            elapsed_g += (uint64_t)(uint32_t)(now - last_tick);
            last_tick = now;
            if ((int32_t)(last_tick - deadline) >= 0) { timeouts_g++; break; }
         }
         if (overruns_g)
            break;
      }

      if (!LEAVE_RUNNING_AFTER_REPORT) {
         for (uint32_t c = 0U; c < NCH; c++) {
            if (!active_ch(c))
               continue;
            dma_disable(SP_DMA[c]);
            dma_wait_idle(SP_DMA[c]);
         }
      }
   }

   if (!LEAVE_RUNNING_AFTER_REPORT) {
      for (uint32_t c = 0U; c < NCH; c++) {
         if (!active_ch(c))
            continue;
         dma_stat_before_g[c] = dma_stat_raw(SP_DMA[c]);
         sport_ctl_before_g[c] = MMR(SP_CTL_REG[c]);
         sport_err_before_g[c] = MMR(SP_ERR_REG[c]);
         sport_disable(SP_ID[c], SP_HALF[c]);
         sport_clear_errors(SP_ID[c], SP_HALF[c]);
         sport_err_after_g[c] = MMR(SP_ERR_REG[c]);
      }
   }
   board_som_set_leds(0U);
   uint32_t total_err = errors_g[0] + errors_g[1];
   uint64_t per_ch_bytes = UINT64_MAX;
   uint64_t agg_bytes = 0ULL;
   for (uint32_t c = 0U; c < NCH; c++) {
      if (!check_ch(c))
         continue;
      uint64_t ch_bytes = (uint64_t)got_words_g[c] * 4ULL;
      agg_bytes += ch_bytes;
      if (ch_bytes < per_ch_bytes)
         per_ch_bytes = ch_bytes;
   }
   if (per_ch_bytes == UINT64_MAX)
      per_ch_bytes = 0ULL;
   bool pass = (total_err == 0U && timeouts_g == 0U && overruns_g == 0U &&
                (!check_ch(0U) || got_words_g[0] == TOTAL_WORDS) &&
                (!check_ch(1U) || got_words_g[1] == TOTAL_WORDS));

   put_str("sport_2x agg_bytes="); put_u64(agg_bytes);
   put_str(" per_ch_bytes="); put_u64(per_ch_bytes);
   put_str(" errors0="); put_u32(errors_g[0]);
   put_str(" errors1="); put_u32(errors_g[1]);
   put_str(" firsterr0="); put_i32(firsterr_g[0]);
   put_str(" firsterr1="); put_i32(firsterr_g[1]);
   put_str(" first0=");
   for (uint32_t i = 0U; i < 4U; i++) {
      if (i) uart_putc(',');
      put_hex32(first_got_g[0][i]);
   }
   put_str("/");
   for (uint32_t i = 0U; i < 4U; i++) {
      if (i) uart_putc(',');
      put_hex32(first_exp_g[i]);
   }
   put_str(" first1=");
   for (uint32_t i = 0U; i < 4U; i++) {
      if (i) uart_putc(',');
      put_hex32(first_got_g[1][i]);
   }
   put_str("/");
   for (uint32_t i = 0U; i < 4U; i++) {
      if (i) uart_putc(',');
      put_hex32(first_exp_g[i]);
   }
   put_str(" timeouts="); put_u32(timeouts_g);
   put_str(" overruns="); put_u32(overruns_g);
   put_str(" dma0=0x"); put_hex32(dma_stat_before_g[0]);
   put_str(" dma1=0x"); put_hex32(dma_stat_before_g[1]);
   put_str(" spctl0=0x"); put_hex32(sport_ctl_before_g[0]);
   put_str(" spctl1=0x"); put_hex32(sport_ctl_before_g[1]);
   put_str(" sperr0=0x"); put_hex32(sport_err_before_g[0]);
   put_str("/0x"); put_hex32(sport_err_after_g[0]);
   put_str(" sperr1=0x"); put_hex32(sport_err_before_g[1]);
   put_str("/0x"); put_hex32(sport_err_after_g[1]);
   put_str(pass ? " PASS\r\n" : " FAIL\r\n");

   for (;;) { }
}
