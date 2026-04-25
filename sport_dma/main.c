// SPDX-License-Identifier: MIT
// main.c --- SPORT4 external loopback clock-rate integrity sweep
// Copyright (c) 2026 Jakob Kastelic

// Tests SPORT4 over the EZLITE P13 jumpers at one CLKDIV per
// build. Compile-time -DSWEEP_CLKDIV=N selects the candidate;
// the host-side wrapper rebuilds and submits per candidate.
// Half-A drives ACLK/AFS/AD0 out; the jumpers loop back into
// half-B. Each round fills tx_buf with a fresh LFSR stream,
// runs SPORT4 long enough for several buffer wraps, then
// verifies rx_buf word-for-word with a rotation offset.

#include "board.h"
#include "clocks.h"
#include "dma.h"
#include "sport.h"
#include "timer.h"
#include "uart.h"
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#define BUF_WORDS 4096U
#define N_ROUNDS  4U

#define LFSR_TAP_31     31U
#define LFSR_TAP_21     21U
#define LFSR_SEED_BASE  0xACE1BEEFU
#define LFSR_ROUND_STEP 0x9E3779B9U

#define RX_SENTINEL 0xDEADBEEFU
#define NO_MATCH    0xFFFFFFFFU

#define WARMUP_US                100000U
#define SPORT_FSDIV              31U
#define SPORT_WORD_BITS          32U
#define SPORT_DIV_BITRATE_FACTOR 2U

// Hz / kHz scaling, dwell shaping.
#define HZ_PER_KHZ      1000U
#define US_PER_MS_LOCAL 1000U
#define DWELL_BUF_FILLS 4U    // dwell = 4 x time-to-fill one buffer
#define DWELL_FLOOR_US  5000U // never sleep less than 5 ms per round

#ifndef SWEEP_CLKDIV
#define SWEEP_CLKDIV 91U
#endif

#pragma section("seg_l2_bss", NO_INIT)
static uint32_t tx_buf[BUF_WORDS];
#pragma section("seg_l2_bss", NO_INIT)
static uint32_t rx_buf[BUF_WORDS];

static struct sport_dsp_cfg tx_master_cfg = {
    .word_bits     = SPORT_WORD_BITS,
    .clkdiv        = SWEEP_CLKDIV,
    .fsdiv         = SPORT_FSDIV,
    .is_tx         = true,
    .internal_clk  = true,
    .internal_fs   = true,
    .late_fs       = true,
    .data_indep_fs = true,
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

static uint32_t udiv32(uint32_t n, uint32_t d)
{
   if (d == 0U)
      return 0U;
   uint32_t q = 0;
   while (n >= d) {
      n -= d;
      q++;
   }
   return q;
}

static void fill_pattern(uint32_t seed)
{
   uint32_t state = seed;
   for (uint32_t w = 0; w < BUF_WORDS; w++) {
      state =
          (state << 1U) | (((state >> LFSR_TAP_31) ^ (state >> LFSR_TAP_21) ^
                            (state >> 1U) ^ state) &
                           1U);
      tx_buf[w] = state;
      rx_buf[w] = RX_SENTINEL;
   }
}

static uint32_t verify(void)
{
   uint32_t first = rx_buf[0];
   uint32_t off   = NO_MATCH;
   for (uint32_t i = 0; i < BUF_WORDS; i++) {
      if (tx_buf[i] == first) {
         off = i;
         break;
      }
   }
   if (off == NO_MATCH)
      return BUF_WORDS;
   uint32_t mismatches    = 0U;
   volatile uint32_t *vmm = &mismatches;
   for (uint32_t i = 0; i < BUF_WORDS; i++) {
      uint32_t idx = i + off;
      if (idx >= BUF_WORDS)
         idx -= BUF_WORDS;
      if (tx_buf[idx] != rx_buf[i])
         *vmm = *vmm + 1U;
   }
   return mismatches;
}

int main(void)
{
   static const struct clocks_cfg clk = BOARD_CLOCKS_CFG;
   clocks_init(&clk);
   uart_init(BOARD_BAUD_DIV);
   timer_init();
   board_som_init(0U);
   delay_us(WARMUP_US);

   uint32_t clkdiv = SWEEP_CLKDIV;
   uint32_t bit_hz =
       udiv32(BOARD_SCLK_HZ, SPORT_DIV_BITRATE_FACTOR * (clkdiv + 1U));
   printf("\r\nsport4_ext clkdiv=%u bit_hz=%u\r\n", (unsigned)clkdiv,
          (unsigned)bit_hz);

   // Compute dwell as 4x time-to-fill BUF_WORDS at this rate.
   // bits = BUF_WORDS*32; time_us = bits*1e6/bit_hz; dwell = 4x.
   // Reorder to avoid 32-bit overflow: bit_khz = bit_hz/1000.
   uint32_t bits_per_buf = BUF_WORDS * SPORT_WORD_BITS;
   uint32_t bit_khz      = udiv32(bit_hz, HZ_PER_KHZ);
   if (bit_khz == 0U)
      bit_khz = 1U;
   uint32_t us_per_buf = udiv32(bits_per_buf * US_PER_MS_LOCAL, bit_khz);
   uint32_t dwell_us   = us_per_buf * DWELL_BUF_FILLS;
   if (dwell_us < DWELL_FLOOR_US)
      dwell_us = DWELL_FLOOR_US;

   uint32_t total_mismatches = 0U;
   volatile uint32_t *vtotal = &total_mismatches;
   uint32_t total_bytes      = 0U;

   // Configure SPORT4 external pins ONCE outside the round loop;
   // tearing down and bringing back up between rounds wedged the
   // peripheral in earlier tests.
#ifdef USE_INTERNAL
   sport_install_internal_loopback(SPORT_ID_4);
#else
   sport_enable_external_pins(SPORT_ID_4);
#endif
   sport_dsp_serial_init(SPORT_ID_4, SPORT_HALF_A, &tx_master_cfg);
   sport_dsp_serial_init(SPORT_ID_4, SPORT_HALF_B, &rx_slave_cfg);

   for (uint32_t r = 0; r < N_ROUNDS; r++) {
      uint32_t seed = LFSR_SEED_BASE ^ (LFSR_ROUND_STEP * r);
      fill_pattern(seed);

      dma_autobuffer_config(DMA_CH_SPORT4_A,
                            (struct dma_buf){tx_buf, BUF_WORDS},
                            DMA_DIR_TX_FROM_MEM);
      dma_autobuffer_config(DMA_CH_SPORT4_B,
                            (struct dma_buf){rx_buf, BUF_WORDS},
                            DMA_DIR_RX_TO_MEM);
      dma_enable(DMA_CH_SPORT4_A);
      dma_enable(DMA_CH_SPORT4_B);
      sport_enable(SPORT_ID_4, SPORT_HALF_B);
      sport_enable(SPORT_ID_4, SPORT_HALF_A);
      sport_clear_errors(SPORT_ID_4, SPORT_HALF_A);
      sport_clear_errors(SPORT_ID_4, SPORT_HALF_B);

      delay_us(dwell_us);

      dma_disable(DMA_CH_SPORT4_B);
      dma_disable(DMA_CH_SPORT4_A);
      sport_disable(SPORT_ID_4, SPORT_HALF_A);
      sport_disable(SPORT_ID_4, SPORT_HALF_B);

      uint32_t mm = verify();
      total_bytes += BUF_WORDS * 4U;
      *vtotal = *vtotal + mm;
      printf("r=%u seed=%x mm=%u\r\n", (unsigned)r, (unsigned)seed,
             (unsigned)mm);
   }

   if (total_mismatches == 0U) {
      printf("clkdiv=%u %u Hz PASS bytes=%u\r\n", (unsigned)clkdiv,
             (unsigned)bit_hz, (unsigned)total_bytes);
   } else {
      printf("clkdiv=%u %u Hz FAIL bytes=%u mm=%u\r\n", (unsigned)clkdiv,
             (unsigned)bit_hz, (unsigned)total_bytes,
             (unsigned)total_mismatches);
   }

   for (;;) {
   }
}
