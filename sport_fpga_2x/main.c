// SPDX-License-Identifier: MIT
// main.c --- TWO SPORTs in parallel (SPORT0B + SPORT4B), each a long
// PRBS31 DMA receiver. The FPGA streams N synchronized PRBS31 channels
// off one PLL; this verifies both and reports the AGGREGATE (2x) rate.
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

#ifndef TOTAL_WORDS
#define TOTAL_WORDS 16777216U   // per channel (64 MB); aggregate = NCH x this
#endif
#define HALF_WORDS        1024U
#define START_TIMEOUT     1000U
#define START_WAIT_LOOPS  2000000U
#define PRBS31_SEED       0x7FFFFFFFU
#define SCLK_HZ_64        93750000ULL
#define NCH               2U
#ifndef PINPOINT_START
#define PINPOINT_START    0U   // 1 = skip SPORT0 (diagnostic)
#endif

static const enum sport_id    SP_ID[NCH]  = { SPORT_ID_0, SPORT_ID_4 };
static const enum dma_channel SP_DMA[NCH] = { DMA_CH_SPORT0_B, DMA_CH_SPORT4_B };

#pragma section("seg_l2_bss", NO_INIT)
static uint32_t rx_buf[NCH][2][HALF_WORDS];
#pragma section("seg_l2_bss", NO_INIT)
static struct dma_dscl rx_desc[NCH][2];

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

static uint64_t udiv64(uint64_t n, uint64_t d)
{
   uint64_t q = 0ULL, r = 0ULL;
   for (int i = 63; i >= 0; i--) {
      r = (r << 1) | ((n >> i) & 1ULL);
      if (r >= d) { r -= d; q |= (1ULL << i); }
   }
   return q;
}

static void put_u64(uint64_t v)
{
   char b[20]; int n = 0;
   if (!v) { uart_putc('0'); return; }
   while (v) {
      uint64_t q = udiv64(v, 10ULL);
      b[n++] = (char)('0' + (uint32_t)(v - q * 10ULL));
      v = q;
   }
   while (n) uart_putc(b[--n]);
}

static inline uint32_t prbs31_word(uint32_t *state)
{
   uint32_t s = *state, w = 0U;
   for (int i = 0; i < 32; i++) {
      uint32_t nb = ((s >> 30) ^ (s >> 27)) & 1U;
      s = ((s << 1) | nb) & 0x7FFFFFFFU;
      w = (w << 1) | nb;
   }
   *state = s;
   return w;
}

static bool discard_until_stream_gap(enum sport_id id)
{
   uint32_t scratch = 0U;
   bool saw = false;
   while (sport_read(id, SPORT_HALF_B, &scratch, START_TIMEOUT) == 0) { }
   for (uint32_t i = 0U; i < START_WAIT_LOOPS; i++) {
      if (sport_read(id, SPORT_HALF_B, &scratch, START_TIMEOUT) == 0)
         saw = true;
      else if (saw)
         return true;
   }
   return false;
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

static void check_half(const uint32_t *buf, uint32_t *prbs, uint32_t *errors,
                       int32_t *firsterr, uint32_t base)
{
   for (uint32_t i = 0U; i < HALF_WORDS; i++) {
      uint32_t expected = prbs31_word(prbs);
      if (buf[i] != expected) {
         if (*firsterr < 0) *firsterr = (int32_t)(base + i);
         if (*errors != UINT32_MAX) (*errors)++;
      }
   }
}

static struct sport_dsp_cfg rx_slave_cfg = {
    .word_bits = 32, .clkdiv = 0, .fsdiv = 0, .is_tx = false,
    .internal_clk = false, .internal_fs = false, .late_fs = true,
    .data_indep_fs = false, .sample_rising = true,
};

int main(void)
{
   static const struct clocks_cfg clk = BOARD_CLOCKS_CFG;
   clocks_init(&clk);
   uart_init(BOARD_BAUD_DIV);
   timer_init();
   board_som_init(0U);

   put_str("\r\nsport_2x boot prbs31 seed=0x7FFFFFFF channels=2 (SPORT0B+SPORT4B)\r\n");

   for (uint32_t c = PINPOINT_START; c < NCH; c++) {
      sport_enable_external_pins(SP_ID[c]);
      sport_install_external_loopback(SP_ID[c]);
      sport_dsp_serial_init(SP_ID[c], SPORT_HALF_B, &rx_slave_cfg);
      sport_clear_errors(SP_ID[c], SPORT_HALF_B);
      sport_enable(SP_ID[c], SPORT_HALF_B);
   }

   uint32_t errors[NCH]   = {0U, 0U};
   uint32_t prbs[NCH]     = {PRBS31_SEED, PRBS31_SEED};
   int32_t  firsterr[NCH] = {-1, -1};
   uint32_t timeouts = 0U, overruns = 0U, got_words = 0U;
   uint64_t elapsed = 0ULL;

   // Sync on SPORT4's idle gap; both channels share the FPGA PLL so they
   // are in the same phase. Arm both DMAs in the idle window so they
   // capture the pattern aligned from word 0.
   if (!discard_until_stream_gap(SPORT_ID_4)) {
      timeouts++;
   } else {
      put_str("SYNCED\r\n");
      for (uint32_t c = PINPOINT_START; c < NCH; c++)
         dma_pingpong_rx_config(SP_DMA[c], rx_buf[c][0], rx_buf[c][1],
                                HALF_WORDS, rx_desc[c]);
      put_str("DMACFG\r\n");
      {
         uint32_t t1 = timer_ticks();
         for (volatile uint32_t z = 0U; z < 2000000U; z++) { }
         uint32_t t2 = timer_ticks();
         put_str("TMR delta="); put_u32(t2 - t1); put_str("\r\n");
      }
      uint32_t last_tick = timer_ticks();

      // SPORT4 (index 1) is the timing master -- its wrap logic is the
      // proven single-channel path. Both channels share the FPGA PLL and
      // were armed in the same idle window, so when SPORT4's half is
      // ready SPORT0's identical half is too; verify both buffers.
      for (uint32_t half = 0U; half < (TOTAL_WORDS / HALF_WORDS); half++) {
         uint32_t deadline = last_tick + (BOARD_SCLK_HZ * 2U);
         while (!dma_wrap_check(DMA_CH_SPORT4_B)) {
            uint32_t now = timer_ticks();
            elapsed += (uint64_t)(uint32_t)(now - last_tick);
            last_tick = now;
            if ((int32_t)(last_tick - deadline) >= 0) { timeouts++; break; }
         }
         if (timeouts) break;

         uint32_t completed = half & 1U;
         if (dma_which_half(DMA_CH_SPORT4_B, rx_buf[1][0], rx_buf[1][1]) == completed) {
            overruns++;
            break;
         }
         for (uint32_t c = 0U; c < NCH; c++)
            check_half(completed == 0U ? rx_buf[c][0] : rx_buf[c][1],
                       &prbs[c], &errors[c], &firsterr[c], got_words);
         got_words += HALF_WORDS;
         uint32_t now = timer_ticks();
         elapsed += (uint64_t)(uint32_t)(now - last_tick);
         last_tick = now;
      }

      for (uint32_t c = 0U; c < NCH; c++) {
         dma_disable(SP_DMA[c]);
         dma_wait_idle(SP_DMA[c]);
      }
   }

   for (uint32_t c = 0U; c < NCH; c++)
      sport_disable(SP_ID[c], SPORT_HALF_B);

   uint32_t total_err = errors[0] + errors[1];
   uint64_t per_ch_bytes = (uint64_t)got_words * 4ULL;
   uint64_t agg_bytes = per_ch_bytes * (uint64_t)NCH;
   uint64_t agg_rate = 0ULL;
   if (elapsed)
      agg_rate = udiv64(agg_bytes * 8ULL * SCLK_HZ_64, elapsed);

   bool pass = (total_err == 0U && timeouts == 0U && overruns == 0U &&
                got_words == TOTAL_WORDS);

   put_str("sport_2x agg_bytes="); put_u64(agg_bytes);
   put_str(" per_ch_bytes="); put_u64(per_ch_bytes);
   put_str(" errors0="); put_u32(errors[0]);
   put_str(" errors1="); put_u32(errors[1]);
   put_str(" timeouts="); put_u32(timeouts);
   put_str(" overruns="); put_u32(overruns);
   put_str(" agg_rate_bps="); put_u64(agg_rate);
   put_str(pass ? " PASS\r\n" : " FAIL\r\n");

   for (;;) { }
   return 0;
}
