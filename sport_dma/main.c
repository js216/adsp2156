// SPDX-License-Identifier: MIT
// main.c --- SPORT DMA loopback integrity + throughput test
// Copyright (c) 2026 Jakob Kastelic

// Runs all eight SPORT top modules in purely internal loopback
// at the maximum supported bit rate. Each module's half A acts
// as master transmitter, half B as slave receiver; an
// autobuffer DMA feeds each TX half from tx_buf[s] and drains
// each RX half into rx_buf[s]. All routing -- clocks, frame
// syncs, data, and the pin-buffer pass-through for SPORT3 /
// SPORT7 -- is installed by sport_install_internal_loopback()
// and lives inside common/sport.c.
//
// The main() flow has two phases:
//
//   1. Verify: run a few buffer wraps, freeze rx_buf by
//      disabling RX DMAs first (before TX or the SPORT halves)
//      so no drain garbage is written on top, then for each
//      channel find the rotation offset that lines up rx_buf[0]
//      with the LFSR-filled tx_buf and confirm every other
//      word matches.
//
//   2. Throughput: re-enable everything and report per-second
//      TX / full-duplex rates by sampling ADDR_CUR over a
//      short 50 us window. Any sticky DERRPRI bit surfaces as
//      a set bit in err=.

#include "board.h"
#include "clocks.h"
#include "dma.h"
#include "sport.h"
#include "timer.h"
#include "uart.h"
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#define BUF_WORDS 2048U
#define N_SPORTS  8

// 32-bit LFSR feedback polynomial taps (x^32 + x^22 + x^2 + x^1).
#define LFSR_TAP_31    31U
#define LFSR_TAP_21    21U
#define LFSR_SEED_BASE 0xACE1BEEFU
#define LFSR_SEED_STEP 0x12345U

// RX buffer sentinel: pre-filled so the verify phase can tell
// untouched words from DMA-written ones.
#define RX_SENTINEL 0xDEADBEEFU

// Rotation-search sentinel: indicates tx_buf has no match for
// rx_buf[0].
#define NO_MATCH 0xFFFFFFFFU

// Timing constants.
#define WARMUP_US        100000U
#define TEST_DURATION_US 10000U

// Throughput reporting constants.
#define THROUGHPUT_WINDOW_DIV 20000U
#define FULL_DUPLEX_FACTOR    2U
#define BYTES_PER_WORD        4U
#define KILO                  1000U
#define REPORT_INTERVAL_DIV   2U

static void throughput_loop(void);

static uint32_t wrap_delta(const uint32_t end, const uint32_t start,
                           const uint32_t wrap)
{
   return (end >= start) ? (end - start) : (wrap - (start - end));
}

// TX / RX buffers land in L2 SRAM via the seg_l2_bss NO_INIT
// section so the DDE channels can reach them through the
// system fabric without the L1 MP alias dance, and the boot
// loader doesn't have to zero-fill 64+ KB at startup.
#pragma section("seg_l2_bss", NO_INIT)
static uint32_t tx_buf[N_SPORTS][BUF_WORDS];
#pragma section("seg_l2_bss", NO_INIT)
static uint32_t rx_buf[N_SPORTS][BUF_WORDS];

// Per-SPORT binding: which SPORT id each index of the buffers
// and DMA channels corresponds to. The sport id drives routing
// selection inside common/sport.c; the two DMA channel ids say
// which DDE lane feeds / drains this module's two halves.
struct sport_binding {
   enum sport_id id;
   enum dma_channel tx_dma;
   enum dma_channel rx_dma;
};

static const struct sport_binding bindings[N_SPORTS] = {
    {SPORT_ID_0, DMA_CH_SPORT0_A, DMA_CH_SPORT0_B},
    {SPORT_ID_1, DMA_CH_SPORT1_A, DMA_CH_SPORT1_B},
    {SPORT_ID_2, DMA_CH_SPORT2_A, DMA_CH_SPORT2_B},
    {SPORT_ID_3, DMA_CH_SPORT3_A, DMA_CH_SPORT3_B},
    {SPORT_ID_4, DMA_CH_SPORT4_A, DMA_CH_SPORT4_B},
    {SPORT_ID_5, DMA_CH_SPORT5_A, DMA_CH_SPORT5_B},
    {SPORT_ID_6, DMA_CH_SPORT6_A, DMA_CH_SPORT6_B},
    {SPORT_ID_7, DMA_CH_SPORT7_A, DMA_CH_SPORT7_B},
};

#define SPORT_CLKDIV    1U
#define SPORT_FSDIV     31U
#define SPORT_WORD_BITS 32U

static const struct sport_dsp_cfg tx_master_cfg = {
    .word_bits     = SPORT_WORD_BITS,
    .clkdiv        = SPORT_CLKDIV,
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

// Fill tx_buf[s] with a per-sport LFSR stream and rx_buf[s]
// with a sentinel so the verify phase can tell untouched words
// from DMA-written ones.
static void fill_test_patterns(void)
{
   for (uint32_t s = 0; s < N_SPORTS; s++) {
      uint32_t state = LFSR_SEED_BASE + (s * LFSR_SEED_STEP);
      for (uint32_t w = 0; w < BUF_WORDS; w++) {
         state =
             (state << 1U) | (((state >> LFSR_TAP_31) ^ (state >> LFSR_TAP_21) ^
                               (state >> 1U) ^ state) &
                              1U);
         tx_buf[s][w] = state;
         rx_buf[s][w] = RX_SENTINEL;
      }
   }
}

// Install routing, configure both halves of every SPORT, and
// hand the two DMA channels the right buffers. After this
// call every channel is ready to run -- the caller still has
// to call dma_enable / sport_enable to turn the stream on.
static void build_loopback_topology(void)
{
   for (uint32_t s = 0; s < N_SPORTS; s++) {
      sport_install_internal_loopback(bindings[s].id);
      sport_dsp_serial_init(bindings[s].id, SPORT_HALF_A, &tx_master_cfg);
      sport_dsp_serial_init(bindings[s].id, SPORT_HALF_B, &rx_slave_cfg);
      dma_autobuffer_config(bindings[s].tx_dma,
                            (struct dma_buf){tx_buf[s], BUF_WORDS},
                            DMA_DIR_TX_FROM_MEM);
      dma_autobuffer_config(bindings[s].rx_dma,
                            (struct dma_buf){rx_buf[s], BUF_WORDS},
                            DMA_DIR_RX_TO_MEM);
   }
}

// Start everything in the order needed to avoid initial
// underruns: DMAs first so the TX FIFOs are preloaded by the
// time the SPORT starts clocking, then the SPORTs themselves,
// then a pass to clear any sticky error bits that the brief
// startup race produced.
static void start_all(void)
{
   for (uint32_t s = 0; s < N_SPORTS; s++) {
      dma_enable(bindings[s].tx_dma);
      dma_enable(bindings[s].rx_dma);
   }
   for (uint32_t s = 0; s < N_SPORTS; s++) {
      sport_enable(bindings[s].id, SPORT_HALF_B);
      sport_enable(bindings[s].id, SPORT_HALF_A);
   }
   for (uint32_t s = 0; s < N_SPORTS; s++) {
      sport_clear_errors(bindings[s].id, SPORT_HALF_A);
      sport_clear_errors(bindings[s].id, SPORT_HALF_B);
   }
}

// Freeze rx_buf by stopping RX DMAs first (before TX or the
// SPORT halves) so the snapshot is a consistent image of what
// the RX pipeline captured up to the freeze instant, with no
// drain garbage written on top.
static void stop_all(void)
{
   for (uint32_t s = 0; s < N_SPORTS; s++) {
      dma_disable(bindings[s].rx_dma);
   }
   for (uint32_t s = 0; s < N_SPORTS; s++) {
      dma_disable(bindings[s].tx_dma);
   }
   for (uint32_t s = 0; s < N_SPORTS; s++) {
      sport_disable(bindings[s].id, SPORT_HALF_A);
      sport_disable(bindings[s].id, SPORT_HALF_B);
   }
}

int main(void)
{
   static const struct clocks_cfg clk = BOARD_CLOCKS_CFG;
   clocks_init(&clk);
   uart_init(BOARD_BAUD_DIV);
   timer_init();
   // Burn 100 ms before the first print so the test rig's
   // UART ring has warmed up; otherwise the first ~100 bytes
   // of output get dropped and the banner is never captured.
   delay_us(WARMUP_US);
   printf("\r\nsport_dma: %x-pair DMA loopback starting\r\n",
          (uint32_t)N_SPORTS);

   fill_test_patterns();
   build_loopback_topology();
   start_all();

   // Let the autobuffer loop run for several complete buffer
   // wraps, then stop in the correct order and walk rx_buf
   // against tx_buf for each channel.
   delay_us(TEST_DURATION_US);
   stop_all();

   // Walk rx_buf[s] against tx_buf[s] for each channel,
   // finding the rotation offset where rx_buf[0] lines up
   // with the LFSR stream and verifying every subsequent
   // word. cc21k -O1 miscompiles this whole block when it
   // lives inside a helper function -- the struct-member
   // lookup for bindings[s].id and/or the indexed 2D buffer
   // access turns into an infinite loop on s>=4. Keeping it
   // inlined here with sid stashed in a local up front is
   // the known-good shape.
   for (uint32_t s = 0; s < N_SPORTS; s++) {
      uint32_t sid   = (uint32_t)bindings[s].id;
      uint32_t first = rx_buf[s][0];

      uint32_t off = NO_MATCH;
      for (uint32_t i = 0; i < BUF_WORDS; i++) {
         if (tx_buf[s][i] == first) {
            off = i;
            break;
         }
      }
      if (off == NO_MATCH) {
         printf("sport%x FAIL no-match rx[0]=%x\r\n", sid, first);
         continue;
      }

      uint32_t mismatches = 0;
      uint32_t first_bad  = 0;
      uint32_t first_exp  = 0;
      uint32_t first_got  = 0;
      for (uint32_t i = 0; i < BUF_WORDS; i++) {
         uint32_t idx = i + off;
         if (idx >= BUF_WORDS)
            idx -= BUF_WORDS;
         uint32_t exp = tx_buf[s][idx];
         uint32_t got = rx_buf[s][i];
         if (exp != got) {
            if (mismatches == 0) {
               first_bad = i;
               first_exp = exp;
               first_got = got;
            }
            mismatches++;
         }
      }
      if (mismatches == 0) {
         printf("sport%x PASS off=%x words=%x\r\n", sid, off,
                (uint32_t)BUF_WORDS);
      } else {
         printf("sport%x FAIL off=%x n=%x @%x exp=%x got=%x\r\n", sid, off,
                mismatches, first_bad, first_exp, first_got);
      }
   }

   // Re-arm everything for the continuous throughput phase.
   build_loopback_topology();
   start_all();
   throughput_loop();
   for (;;) {
   }
}

// Continuous throughput measurement. Runs forever, printing
// aggregate TX rate and full-duplex rate every ~500 ms. Broken
// out of main() to keep cognitive complexity under the
// clang-tidy threshold. All struct-member lookups are resolved
// into plain local arrays before the hot loop, which avoids
// the cc21k -O1 miscompile with bindings[s].member access.
static void throughput_loop(void)
{
   enum dma_channel tx_chs[N_SPORTS];
   enum sport_id sport_ids[N_SPORTS];
   for (uint32_t s = 0; s < N_SPORTS; s++) {
      tx_chs[s]    = bindings[s].tx_dma;
      sport_ids[s] = bindings[s].id;
   }
   const uint32_t buf_bytes = BUF_WORDS * BYTES_PER_WORD;

   // Short measurement window: TIMER_SCLK_HZ / THROUGHPUT_WINDOW_DIV
   // = 50 us, shorter than one 2048-word buffer wrap at CLKDIV=1,
   // so we never have to wrap-correct more than once per sample.
   const uint32_t short_win = TIMER_SCLK_HZ / THROUGHPUT_WINDOW_DIV;
   for (;;) {
      uint32_t ws = timer_ticks();
      uint32_t start_addr[N_SPORTS];
      for (uint32_t s = 0; s < N_SPORTS; s++) {
         start_addr[s] = dma_addr_cur(tx_chs[s]);
      }
      while ((timer_ticks() - ws) < short_win) {
      }
      uint32_t end_addr[N_SPORTS];
      for (uint32_t s = 0; s < N_SPORTS; s++) {
         end_addr[s] = dma_addr_cur(tx_chs[s]);
      }

      // Per-channel accumulate. The original hand-unrolled version
      // worked around a cc21k -O1 loop-body miscompile; the local
      // arrays above sidestep that issue so a plain loop is safe.
      uint32_t total_bytes = 0;
      for (uint32_t s = 0; s < N_SPORTS; s++) {
         total_bytes += wrap_delta(end_addr[s], start_addr[s], buf_bytes);
      }
      uint32_t total_words = total_bytes >> 2U;

      uint32_t tx_wps = total_words * THROUGHPUT_WINDOW_DIV;
      uint32_t fd_wps = tx_wps * FULL_DUPLEX_FACTOR;
      uint32_t tx_mbs = (tx_wps / KILO) * BYTES_PER_WORD / KILO;
      uint32_t fd_mbs = (fd_wps / KILO) * BYTES_PER_WORD / KILO;

      uint32_t rs = timer_ticks();
      while ((timer_ticks() - rs) < (TIMER_SCLK_HZ / REPORT_INTERVAL_DIV)) {
      }

      uint32_t err_mask = 0;
      for (uint32_t s = 0; s < N_SPORTS; s++) {
         if (sport_has_error(sport_ids[s], SPORT_HALF_A) ||
             sport_has_error(sport_ids[s], SPORT_HALF_B)) {
            err_mask |= (1U << s);
         }
      }

      printf("tx %x w/s (%x MB/s)  fd %x w/s (%x MB/s)  err=%x\r\n", tx_wps,
             tx_mbs, fd_wps, fd_mbs, err_mask);
   }
}
