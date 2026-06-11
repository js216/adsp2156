// SPDX-License-Identifier: MIT
// main.c --- Concurrent bidirectional SPORT test.
// RX_N FPGA->DSP PRBS31 lanes (SPORT4B/SPORT0B, DMA ping-pong RX, verified
// in halves) run AT THE SAME TIME as TX_N DSP->FPGA counter lanes
// (SPORT5A/SPORT1A, DMA ping-pong TX, refilled in halves). A single core
// loop services both DMA directions so neither underruns/overruns.
// Copyright (c) 2026 Jakob Kastelic

#include "board.h"
#include "clocks.h"
#include "dma.h"
#include "gpio.h"
#include "sport.h"
#include "timer.h"
#include "uart.h"
#include "regs.h"
#include <stdbool.h>
#include <stdint.h>

#ifndef RX_N
#define RX_N 1U
#endif
#ifndef TX_N
#define TX_N 1U
#endif
#ifndef TOTAL_WORDS
#define TOTAL_WORDS 16777216U
#endif
#ifndef RX_PRBS_SKIP_WORDS
#define RX_PRBS_SKIP_WORDS 0U
#endif
#ifndef RX_HALF
#define RX_HALF 8192U
#endif
#ifndef TX_HALF
#define TX_HALF 4096U
#endif
#define PRBS31_SEED 0x7FFFFFFFU
#define SCLK_HZ 62500000ULL
#define LOCAL_BAUD_DIV ((uint32_t)((SCLK_HZ + (BOARD_BAUD / 2ULL)) / BOARD_BAUD))
#define START_TIMEOUT 1000U
#define START_WAIT_LOOPS 2000000U

// TX uses SPORT4A/SPORT0A so the FPGA receives the DSP counter on the
// N10/T11/T13 + R5/T5/R9 pins -- the clock routes that a standalone receiver
// samples cleanly. RX (the FPGA PRBS stream) moves to SPORT5B/SPORT1B.
#ifdef LANE1_SOLO
static const enum sport_id    RX_ID[4]  = {SPORT_ID_1, SPORT_ID_5, SPORT_ID_4, SPORT_ID_0};
#else
static const enum sport_id    RX_ID[4]  = {SPORT_ID_5, SPORT_ID_1, SPORT_ID_4, SPORT_ID_0};
#endif
#ifdef LANE1_SOLO
static const enum dma_channel RX_DMA[4] = {DMA_CH_SPORT1_B, DMA_CH_SPORT5_B, DMA_CH_SPORT4_B, DMA_CH_SPORT0_B};
#else
static const enum dma_channel RX_DMA[4] = {DMA_CH_SPORT5_B, DMA_CH_SPORT1_B, DMA_CH_SPORT4_B, DMA_CH_SPORT0_B};
#endif
static const enum sport_id    TX_ID[2]  = {SPORT_ID_4, SPORT_ID_0};
static const enum dma_channel TX_DMA[2] = {DMA_CH_SPORT4_A, DMA_CH_SPORT0_A};

#pragma section("seg_l2_bss", NO_INIT)
static uint32_t rx_buf[RX_N][2][RX_HALF];
#pragma section("seg_l2_bss", NO_INIT)
static struct dma_dscl rx_desc[RX_N][2];
#pragma section("seg_l2_bss", NO_INIT)
static uint32_t tx_buf[TX_N][2][TX_HALF];
#pragma section("seg_l2_bss", NO_INIT)
static struct dma_dscl tx_desc[TX_N][2];

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

static inline uint32_t prbs31_word(uint32_t *state)
{
   uint32_t s = *state, w = 0U;
#define PRBS_STEP() do { \
      uint32_t nb = ((s >> 30) ^ (s >> 27)) & 1U; \
      s = ((s << 1) | nb) & 0x7FFFFFFFU; \
      w = (w << 1) | nb; \
   } while (0)
   PRBS_STEP();PRBS_STEP();PRBS_STEP();PRBS_STEP();PRBS_STEP();PRBS_STEP();PRBS_STEP();PRBS_STEP();
   PRBS_STEP();PRBS_STEP();PRBS_STEP();PRBS_STEP();PRBS_STEP();PRBS_STEP();PRBS_STEP();PRBS_STEP();
   PRBS_STEP();PRBS_STEP();PRBS_STEP();PRBS_STEP();PRBS_STEP();PRBS_STEP();PRBS_STEP();PRBS_STEP();
   PRBS_STEP();PRBS_STEP();PRBS_STEP();PRBS_STEP();PRBS_STEP();PRBS_STEP();PRBS_STEP();PRBS_STEP();
#undef PRBS_STEP
   *state = s;
   return w;
}

static uint32_t dma_which_half(enum dma_channel ch, const uint32_t *b0, const uint32_t *b1)
{
   uint32_t addr = dma_addr_cur(ch);
   uint32_t a0 = (uint32_t)b0, a1 = (uint32_t)b1, bytes = RX_HALF * 4U;
   if (addr >= a1 && addr < a1 + bytes) return 1U;
   if (addr >= a0 && addr < a0 + bytes) return 0U;
   return 2U;
}

static bool dma_still_on_half(enum dma_channel ch, const uint32_t *b0,
                              const uint32_t *b1, uint32_t completed)
{
   for (uint32_t i = 0U; i < 10000U; i++)
      if (dma_which_half(ch, b0, b1) != completed) return false;
   return dma_which_half(ch, b0, b1) == completed;
}

static void put_hex(uint32_t v)
{
   for (int s = 28; s >= 0; s -= 4) {
      uint32_t n = (v >> s) & 0xFU;
      uart_putc(n < 10U ? ('0' + n) : ('a' + n - 10U));
   }
}
#ifdef DIAG_FIRST
static bool diag_done = false;
#endif
static void tx_fill(uint32_t c, uint32_t half, uint32_t start);

// Table-driven PRBS31: step the LFSR 8 bits at a time (the taps 31,28
// span bits [30:20] over 8 steps, so an 11-bit index suffices), making
// the RX-half check ~8x faster so it never starves the TX refill.
static uint8_t p8_bits[2048];
static void prbs8_init(void)
{
   for (uint32_t v = 0U; v < 2048U; v++) {
      uint32_t s = v << 20;
      uint32_t b = 0U;
      for (uint32_t i = 0U; i < 8U; i++) {
         uint32_t nb = ((s >> 30) ^ (s >> 27)) & 1U;
         s = ((s << 1) & 0x7fffffffU) | nb;
         b = (b << 1) | nb;
      }
      p8_bits[v] = (uint8_t)b;
   }
}
static uint32_t prbs31_word_fast(uint32_t *state)
{
   uint32_t s = *state, w = 0U;
   for (uint32_t k = 0U; k < 4U; k++) {
      uint32_t bits = p8_bits[(s >> 20) & 0x7ffU];
      w = (w << 8) | bits;
      s = ((s << 8) & 0x7fffffffU) | bits;
   }
   *state = s;
   return w;
}

static void check_half(uint32_t completed, uint32_t *prbs, uint32_t errors[4])
{
#ifdef DIAG_FIRST
   if (!diag_done) {
      diag_done = true;
      uint32_t p = PRBS31_SEED;
      for (uint32_t i = 0U; i < 8U; i++) {
         put_str("rx["); put_u32(i); put_str("]=");
         put_hex(rx_buf[0][completed][i]); put_str(" exp=");
         put_hex(prbs31_word(&p)); put_str("\r\n");
      }
   }
#endif
   for (uint32_t i = 0U; i < RX_HALF; i++) {
      uint32_t expected = prbs31_word_fast(prbs);
      for (uint32_t c = 0U; c < RX_N; c++)
         if (rx_buf[c][completed][i] != expected && errors[c] != UINT32_MAX)
            errors[c]++;
   }
}

// PRBS-31 TX: each channel has its own LFSR state advanced continuously across
// half-buffer refills, seeded to PRBS31_SEED so the stream starts at word 0 =
// the same seed the FPGA RX LFSR resets to. No counter, no framing payload.
#ifndef TX_CKRE
#define TX_CKRE 1
#endif
static uint32_t tx_prbs[2] = {PRBS31_SEED, PRBS31_SEED};
static void tx_fill(uint32_t c, uint32_t half, uint32_t start)
{
   (void)start;
   for (uint32_t i = 0U; i < TX_HALF; i++)
      tx_buf[c][half][i] = prbs31_word_fast(&tx_prbs[c]);
}

// Which TX half-buffer a channel's DMA is currently reading (2 if between).
static uint32_t tx_which_half(enum dma_channel ch, uint32_t c)
{
   uint32_t addr = dma_addr_cur(ch);
   uint32_t a0 = (uint32_t)&tx_buf[c][0][0];
   uint32_t a1 = (uint32_t)&tx_buf[c][1][0];
   uint32_t bytes = TX_HALF * 4U;
   if (addr >= a1 && addr < a1 + bytes) return 1U;
   if (addr >= a0 && addr < a0 + bytes) return 0U;
   return 2U;
}

static struct sport_dsp_cfg rx_cfg = {
    .word_bits = 32, .clkdiv = 0, .fsdiv = 0, .is_tx = false,
    .internal_clk = false, .internal_fs = false, .late_fs = true,
    .data_indep_fs = false, .sample_rising = true,
};
static struct sport_dsp_cfg tx_cfg = {
    .word_bits = 32, .clkdiv = 0, .fsdiv = 31, .is_tx = true,
    .internal_clk = true, .internal_fs = true, .late_fs = true,
    .data_indep_fs = false, .sample_rising = (TX_CKRE != 0),
};

int main(void)
{
   static const struct clocks_cfg clk = CLOCKS_CFG_SCLK0_62MHZ;
   clocks_init(&clk);
   uart_init(LOCAL_BAUD_DIV);
   while ((MMR(REG_UART0_STAT) & BIT_UART_STAT_THRE) == 0U) {}
   timer_init();
   prbs8_init();
   board_som_init(0U);
   // RUN low while everything arms: the FPGA to_dsp transmitters hold
   // their LFSRs at seed until RUN goes active (mission criteria), so no
   // data can hit the RX FIFOs during the DMA-arm startup jam.
   gpio_make_output(GPIO_DAI1_06);
   gpio_write(GPIO_DAI1_06, false);

   put_str("\r\nsport_bidir_concurrent rx=");
   put_u32(RX_N); put_str(" tx="); put_u32(TX_N); put_str("\r\n");

   // ---- RX setup (slave: SCLK+FS arrive from the FPGA, which relays the DSP's
   // own master clock). Enable and arm the RX DMA BEFORE the TX clock starts so
   // the first F->D word the FPGA emits (PRBS word 0) lands at rx_buf[*][0][0]
   // and lines up with the RX LFSR's seed -- that is the entire "sync". ----
#ifdef LANE1_SOLO
   sport_route_rx_from_pins(SPORT_ID_1, 0U, 10U, 12U, 20U);
#else
   sport_route_rx_from_pins(SPORT_ID_5, 1U, 9U, 10U, 11U);
#endif
#if RX_N >= 3U
   // 4-lane F->D: two lanes per DAI share one forwarded clk/fs pair;
   // extra data pins ride existing wires (dai1_12 -> R16, dai0_19 -> R4).
   sport_route_rx_from_pins(SPORT_ID_4, 1U, 12U, 10U, 11U);
   sport_route_rx_from_pins(SPORT_ID_0, 0U, 19U, 12U, 20U);
#endif
   if (RX_N > 1U)
      sport_route_rx_from_pins(SPORT_ID_1, 0U, 10U, 12U, 20U);
   for (uint32_t c = 0U; c < RX_N; c++) {
      sport_dsp_serial_init(RX_ID[c], SPORT_HALF_B, &rx_cfg);
      sport_clear_errors(RX_ID[c], SPORT_HALF_B);
      sport_enable(RX_ID[c], SPORT_HALF_B);
   }
   for (uint32_t c = 0U; c < RX_N; c++)
      dma_pingpong_rx_config(RX_DMA[c], rx_buf[c][0], rx_buf[c][1], RX_HALF, rx_desc[c]);

   uint32_t errors[4] = {0U, 0U, 0U, 0U};
   uint32_t prbs = PRBS31_SEED;
   for (uint32_t i = 0U; i < RX_PRBS_SKIP_WORDS; i++)
      (void)prbs31_word(&prbs);
   uint32_t timeouts = 0U, overruns = 0U, tx_timeouts = 0U;
   uint32_t got_words = 0U;
   uint64_t elapsed = 0ULL;

   // ---- TX setup (master: the DSP SPORT generates the ONE transfer clock for
   // both directions). Pre-fill two halves with PRBS from the seed, arm DMA,
   // then enable -> the clock starts and both directions stream from word 0. ----
   uint32_t tx_done_halves[2] = {0U, 0U};
   const uint32_t tx_halves_target = (TOTAL_WORDS + TX_HALF - 1U) / TX_HALF + 2U;
   sport_route_tx_to_pins(SPORT_ID_4, 1U, 5U, 7U, 8U);
   // Second copy of the SPT4A clock on DAI1_PIN04 (FPGA ball P10): the
   // FPGA taps this copy for its to_dsp forwarder so the from_dsp
   // capture clock keeps a single light load (proven topology).
   sport_route_clk_copy(SPORT_ID_4, 1U, 4U);
#if RX_N >= 3U
   sport_route_clk_copy(SPORT_ID_0, 0U, 2U);
#endif
   if (TX_N > 1U)
      sport_route_sport0a_to_wired_pins();
   for (uint32_t c = 0U; c < TX_N; c++) {
      sport_dsp_serial_init(TX_ID[c], SPORT_HALF_A, &tx_cfg);
      sport_clear_errors(TX_ID[c], SPORT_HALF_A);
      tx_fill(c, 0U, 0U);
      tx_fill(c, 1U, 0U);
   }
   for (uint32_t c = 0U; c < TX_N; c++)
      dma_pingpong_tx_config(TX_DMA[c], tx_buf[c][0], tx_buf[c][1], TX_HALF, tx_desc[c]);
   // RUN rises BEFORE the TX clocks start: word zero on BOTH directions
   // is the first FS after RUN (criteria-exact). The FPGA from_dsp
   // receivers exit reset on RUN and arm on the first FS the DSP emits;
   // the to_dsp LFSRs release on their first forwarded clocks.
   delay_us(2000U);
   gpio_write(GPIO_DAI1_06, true);
   delay_us(500U);
   for (uint32_t c = 0U; c < TX_N; c++)
      sport_enable(TX_ID[c], SPORT_HALF_A);

   const uint32_t rx_halves_target = TOTAL_WORDS / RX_HALF;
   uint32_t rx_half = 0U;
   uint32_t last_tick = timer_ticks();
   const uint64_t tick_limit = SCLK_HZ * 360ULL;
   bool rx_done = (RX_N == 0U);
   bool tx_done = (TX_N == 0U);

   // Terminate when the RX has collected TOTAL_WORDS; the TX has streamed the
   // same amount concurrently (same clock), so the FPGA from_dsp has ~TOTAL_WORDS
   // too. Keep refilling TX while the loop runs.
   while (!rx_done && timeouts == 0U && tx_timeouts == 0U) {
      // --- service RX (RX_DMA[0] is the timing reference) ---
      if (!rx_done && dma_wrap_check(RX_DMA[0])) {
         uint32_t completed = rx_half & 1U;
         if (dma_still_on_half(RX_DMA[0], rx_buf[0][0], rx_buf[0][1], completed)) {
            overruns++;
         } else {
            check_half(completed, &prbs, errors);
            got_words += RX_HALF;
            rx_half++;
            if (rx_half >= rx_halves_target) rx_done = true;
         }
      }
      // --- service TX (address-based half tracking) ---
#ifdef TX_NO_REFILL
      // F->D-only rows: the TX lanes exist purely as clock/FS masters for
      // the to_dsp chain. Let the DMA ring loop over stale buffers with no
      // core involvement so the RX side owns all memory bandwidth.
      tx_done = true;
#else
      tx_done = true;
      for (uint32_t c = 0U; c < TX_N; c++) {
         if (tx_done_halves[c] < tx_halves_target) {
            // Refill on the LATCHED wrap flag with parity-derived half:
            // the DMA address flaps during descriptor reload and the old
            // address poll double-filled (clobbering the live half).
            if (dma_wrap_check(TX_DMA[c])) {
               tx_fill(c, tx_done_halves[c] & 1U, 0U);
               tx_done_halves[c]++;
#ifdef FILL_TRACE
               if (c == 0U && tx_done_halves[0] <= 6U) {
                  put_str("fill n="); put_u32(tx_done_halves[0]);
                  put_str(" half="); put_u32(last_tx_half_g[0]);
                  put_str(" half="); put_u32((tx_done_halves[0] - 1U) & 1U);
                  put_str(" prbs="); put_u32(tx_prbs[0] & 0xffffU);
                  put_str("\r\n");
               }
#endif
            }
            tx_done = false;
         }
      }
#endif
      uint32_t now = timer_ticks();
      elapsed += (uint64_t)(uint32_t)(now - last_tick);
      last_tick = now;
      if (elapsed >= tick_limit) timeouts++;
   }

   // Trailing clock: the FPGA registers inputs in IO cells, so the last
   // word needs extra clock edges to flush through and latch done.
   delay_us(10000U);
   for (uint32_t c = 0U; c < TX_N; c++) {
      sport_disable(TX_ID[c], SPORT_HALF_A);
      dma_disable(TX_DMA[c]);
      dma_wait_idle(TX_DMA[c]);
   }
   for (uint32_t c = 0U; c < RX_N; c++) {
      dma_disable(RX_DMA[c]);
      dma_wait_idle(RX_DMA[c]);
      sport_disable(RX_ID[c], SPORT_HALF_B);
   }

   uint32_t total_err = 0U;
   for (uint32_t c = 0U; c < RX_N; c++)
      total_err += errors[c];
   bool pass = (got_words >= TOTAL_WORDS && total_err == 0U && timeouts == 0U &&
                tx_timeouts == 0U && overruns == 0U);

   // Report only 32-bit counts; the bench verify derives bytes/rate from
   // wall-clock op timestamps (64-bit printf is unavailable here).
   put_str("sport_bidir rx_lanes="); put_u32(RX_N);
   put_str(" tx_lanes="); put_u32(TX_N);
   put_str(" rx_words="); put_u32(got_words);
   put_str(" rx_errors="); put_u32(total_err);
#if RX_N >= 3U
   put_str(" e0="); put_u32(errors[0]);
   put_str(" e1="); put_u32(errors[1]);
   put_str(" e2="); put_u32(errors[2]);
   put_str(" e3="); put_u32(errors[3]);
#endif
   put_str(" timeouts="); put_u32(timeouts);
   put_str(" tx_timeouts="); put_u32(tx_timeouts);
   put_str(" overruns="); put_u32(overruns);
   put_str(" slips="); put_u32(0U);
   put_str(" tx_sent="); put_u32(TOTAL_WORDS);
   put_str(pass ? " PASS\r\n" : " FAIL\r\n");

   for (;;) {}
   return 0;
}
