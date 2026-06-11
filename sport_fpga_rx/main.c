// SPDX-License-Identifier: MIT
// main.c --- N-lane SPORT half-A master TX into an FPGA receiver, fed by
// ping-pong DMA so the serial wire never underruns.
// Copyright (c) 2026 Jakob Kastelic
//
// SPORT4/0/5/1 half-A are master TX (internal clock + internal frame
// sync, SCLK0 = 62.5 MHz, CLKDIV=0 -> 62.5 Mbit/s/lane). Each lane streams
// PRBS-31 from the shared seed after RUN is asserted.  A two-buffer
// ping-pong DMA channel per lane keeps the SPORT TFIFO fed at line rate;
// the core refills each half with the continuing counter as soon as the
// DMA finishes draining it (poll dma_wrap_check).
//
// The previous polled transmitter could only sustain ~51 Mbit/s/lane and
// underran on long transfers.

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

#ifndef N_WORDS
#define N_WORDS               16777216U   // per lane (64 MB)
#endif
#ifndef NCH
#define NCH                   2U
#endif
#ifndef HALF_WORDS
#define HALF_WORDS            4096U
#endif
#ifndef DATA_INDEP_FS
#define DATA_INDEP_FS         0
#endif
// Progress heartbeat: one "tx h=<halves>" line per PROG_WORDS words sent
// (8 MiB; ~1.1 s at 60 Mbps) so a stalled link is visible within seconds.
#define PROG_WORDS            2097152U
#define PROG_HALVES           (PROG_WORDS / HALF_WORDS)
#if NCH > 1U
#define SHARED_TXBUF          1
#else
#define SHARED_TXBUF          0
#endif
#define SPORT_WORD_BITS       32U
#define SPORT_FSDIV           31U
#ifndef SPORT_SCLK_HZ
#define SPORT_SCLK_HZ         62500000U
#endif
#define SPORT_CLKDIV          0U
#define SPORT_BIT_CLK_HZ      (SPORT_SCLK_HZ / (SPORT_CLKDIV + 1U))
#define AGG_RATE_BPS          (SPORT_BIT_CLK_HZ * NCH)
#define LOCAL_BAUD_DIV        ((SPORT_SCLK_HZ + (BOARD_BAUD / 2U)) / BOARD_BAUD)
#define PRBS31_SEED           0x7fffffffU
#define SPORT_WORDS_PER_S     (SPORT_BIT_CLK_HZ / SPORT_WORD_BITS)
#ifndef TX_TIMEOUT_S
#define TX_TIMEOUT_S          (((N_WORDS + SPORT_WORDS_PER_S - 1U) /          \
                                SPORT_WORDS_PER_S) + 30U)
#endif

static const enum sport_id    TX_ID[4]  = {
    SPORT_ID_4, SPORT_ID_0, SPORT_ID_5, SPORT_ID_1};
static const enum dma_channel TX_DMA[4] = {
    DMA_CH_SPORT4_A, DMA_CH_SPORT0_A, DMA_CH_SPORT5_A, DMA_CH_SPORT1_A};

#ifndef TXBUF_DEFAULT_SECTION
#pragma section("seg_l2_bss", NO_INIT)
#endif
#ifdef VOLATILE_TXBUF
#if SHARED_TXBUF
static volatile uint32_t tx_buf[2][HALF_WORDS];
#else
static volatile uint32_t tx_buf[NCH][2][HALF_WORDS];
#endif
#else
#if SHARED_TXBUF
static uint32_t tx_buf[2][HALF_WORDS];
#else
static uint32_t tx_buf[NCH][2][HALF_WORDS];
#endif
#endif
#pragma section("seg_l2_bss", NO_INIT)
static struct dma_dscl tx_desc[NCH][2];

static void put_str(const char *s) { while (*s != '\0') uart_putc(*s++); }

#ifndef DIAG
#define DIAG 0
#endif
static void diag(const char *s)
{
#if DIAG
   put_str(s); put_str("\r\n");
#else
   (void)s;
#endif
}

static void put_u32(uint32_t v)
{
   static const uint32_t pow10[10] = {
       1000000000U, 100000000U, 10000000U, 1000000U, 100000U,
       10000U,      1000U,      100U,      10U,      1U,
   };
   if (v == 0U) { uart_putc('0'); return; }
   unsigned started = 0U;
   for (unsigned i = 0U; i < 10U; i++) {
      uint32_t d = pow10[i];
      char c = '0';
      while (v >= d) { v -= d; c = (char)(c + 1); }
      if (c != '0') started = 1U;
      if (started != 0U) uart_putc(c);
   }
}

static uint32_t prbs31_word(uint32_t *state)
{
   uint32_t s = *state & PRBS31_SEED;
   uint32_t w = 0U;
#define PRBS31_STEP() do {                         \
      uint32_t bit = ((s >> 30) ^ (s >> 27)) & 1U; \
      w = (w << 1) | bit;                          \
      s = ((s << 1) & PRBS31_SEED) | bit;          \
   } while (0)
   PRBS31_STEP(); PRBS31_STEP(); PRBS31_STEP(); PRBS31_STEP();
   PRBS31_STEP(); PRBS31_STEP(); PRBS31_STEP(); PRBS31_STEP();
   PRBS31_STEP(); PRBS31_STEP(); PRBS31_STEP(); PRBS31_STEP();
   PRBS31_STEP(); PRBS31_STEP(); PRBS31_STEP(); PRBS31_STEP();
   PRBS31_STEP(); PRBS31_STEP(); PRBS31_STEP(); PRBS31_STEP();
   PRBS31_STEP(); PRBS31_STEP(); PRBS31_STEP(); PRBS31_STEP();
   PRBS31_STEP(); PRBS31_STEP(); PRBS31_STEP(); PRBS31_STEP();
   PRBS31_STEP(); PRBS31_STEP(); PRBS31_STEP(); PRBS31_STEP();
#undef PRBS31_STEP
   *state = s;
   return w;
}

static void fill_half(uint32_t c, uint32_t half, uint32_t *state)
{
#if SHARED_TXBUF
   (void)c;
   for (uint32_t i = 0U; i < HALF_WORDS; i++)
      tx_buf[half][i] = prbs31_word(state);
#else
   for (uint32_t i = 0U; i < HALF_WORDS; i++)
      tx_buf[c][half][i] = prbs31_word(state);
#endif
}

#ifndef TX_CKRE
#define TX_CKRE 1
#endif
static struct sport_dsp_cfg tx_master_cfg = {
    .word_bits     = SPORT_WORD_BITS,
    .clkdiv        = SPORT_CLKDIV,
    .fsdiv         = SPORT_FSDIV,
    .is_tx         = true,
    .internal_clk  = true,
    .internal_fs   = true,
    .late_fs       = true,
    .data_indep_fs = (DATA_INDEP_FS != 0),
    .sample_rising = (TX_CKRE != 0),
};

static void route_lane(uint32_t c)
{
   switch (c) {
   case 0U: sport_route_tx_to_pins(SPORT_ID_4, 1U, 5U, 7U, 8U);   break;
   case 1U: sport_route_sport0a_to_wired_pins();                  break;
   case 2U: sport_route_tx_to_pins(SPORT_ID_5, 1U, 9U, 10U, 11U); break;
   default: sport_route_tx_to_pins(SPORT_ID_1, 0U, 10U, 12U, 20U); break;
   }
}

int main(void)
{
   static const struct clocks_cfg clk =
#if SPORT_SCLK_HZ == 60000000U
      CLOCKS_CFG_SCLK0_60MHZ;
#elif SPORT_SCLK_HZ == 60833333U
      CLOCKS_CFG_SCLK0_60833333HZ;
#else
      CLOCKS_CFG_SCLK0_62MHZ;
#endif
   clocks_init(&clk);
   uart_init(LOCAL_BAUD_DIV);
   while ((MMR(REG_UART0_STAT) & BIT_UART_STAT_THRE) == 0U) { }
   timer_init();
   board_som_init(0U);
   gpio_make_output(GPIO_DAI1_06);
   gpio_write(GPIO_DAI1_06, false);
   // Hold RUN low for an unmistakable 60 ms: the FPGA only enables its
   // UART reporting after seeing a >=44 ms continuous low followed by a
   // rise, which protects the shared boot bus from early prints.
   delay_us(60000U);

   put_str("\r\nsport_fpga_rx boot channels=");
   put_u32(NCH);
   put_str(" dma_pingpong\r\n");

#ifdef RUN_TOGGLE_TEST
   // Bisect mode: drive RUN as a slow square wave forever, so the
   // FPGA-side state tracer shows whether the pin actually moves.
#ifdef RUN_TOGGLE_AFTER_ROUTE
   for (uint32_t c = 0U; c < NCH; c++)
      route_lane(c);
   for (uint32_t c = 0U; c < NCH; c++) {
      sport_dsp_serial_init(TX_ID[c], SPORT_HALF_A, &tx_master_cfg);
      sport_clear_errors(TX_ID[c], SPORT_HALF_A);
   }
   put_str("routed ");
#endif
   put_str("run toggle test\r\n");
   for (;;) {
      gpio_write(GPIO_DAI1_06, true);
      delay_us(300000U);
      gpio_write(GPIO_DAI1_06, false);
      delay_us(300000U);
   }
#endif
   for (uint32_t c = 0U; c < NCH; c++)
      route_lane(c);
   diag("routed");
   for (uint32_t c = 0U; c < NCH; c++) {
      sport_dsp_serial_init(TX_ID[c], SPORT_HALF_A, &tx_master_cfg);
      sport_clear_errors(TX_ID[c], SPORT_HALF_A);
   }
   diag("inited");

   // Per-lane PRBS state for the next buffer word, and how many
   // halves have fully drained (== sequential words transmitted / H).
   uint32_t lane_prbs[4];
   uint32_t lane_done_halves[4];
#if SHARED_TXBUF
   uint32_t shared_prbs = PRBS31_SEED;
   fill_half(0U, 0U, &shared_prbs);
   fill_half(0U, 1U, &shared_prbs);
#endif
   for (uint32_t c = 0U; c < NCH; c++) {
      lane_prbs[c] = PRBS31_SEED;
#if !SHARED_TXBUF
      fill_half(c, 0U, &lane_prbs[c]);
      fill_half(c, 1U, &lane_prbs[c]);
#endif
      lane_done_halves[c] = 0U;
   }

   // Start each lane's ping-pong DMA (auto-enables), then RUN, then the
   // SPORT so it begins pulling words out of the TFIFO.
   diag("prefilled");
   for (uint32_t c = 0U; c < NCH; c++) {
#if SHARED_TXBUF
      dma_pingpong_tx_config(TX_DMA[c], (const void *)tx_buf[0],
                             (const void *)tx_buf[1],
                             HALF_WORDS, tx_desc[c]);
#else
      dma_pingpong_tx_config(TX_DMA[c], (const void *)tx_buf[c][0],
                             (const void *)tx_buf[c][1],
                             HALF_WORDS, tx_desc[c]);
#endif
   }
   gpio_write(GPIO_DAI1_06, true);
   // Generous RUN-to-enable gap: the FPGA detects the RUN edge on its
   // always-running 12 MHz clock and re-arms on the first bit-clock
   // ticks; by the time data flows the receiver is freshly armed.
   delay_us(500U);
   for (uint32_t c = 0U; c < NCH; c++)
      sport_enable(TX_ID[c], SPORT_HALF_A);
   diag("enabled");

   // Send two extra halves beyond the requested count so the receiver,
   // which discards a handful of words while it confirms frame lock,
   // still tallies at least N_WORDS sequential words.
   const uint32_t target_halves = (N_WORDS + HALF_WORDS - 1U) / HALF_WORDS + 2U;
   const uint64_t tick_limit = (uint64_t)SPORT_SCLK_HZ * (uint64_t)TX_TIMEOUT_S;
   uint64_t elapsed = 0ULL;
   uint32_t last_tick = timer_ticks();
   uint32_t tx_timeouts = 0U;
   bool all_done = false;

   bool saw_first = false;
   uint32_t diag_last = 0xffffffffU;
   (void)diag_last;
   // The TX descriptor chain always starts at half 0 and alternates 0,1,0,1.
   // Refill the half whose descriptor just completed; polling IRQDONE is much
   // less intrusive than continuously reading ADDR_CUR on the live DMA engine.
   uint32_t refill_half[4];
   for (uint32_t c = 0U; c < NCH; c++)
      refill_half[c] = 0U;
#if SHARED_TXBUF
   uint32_t shared_refill_half = 0U;
   uint32_t shared_half_done[2] = {0U, 0U};
#endif
   while (!all_done && tx_timeouts == 0U) {
      all_done = true;
      for (uint32_t c = 0U; c < NCH; c++) {
         if (lane_done_halves[c] < target_halves) {
            if (dma_wrap_check(TX_DMA[c])) {
#if SHARED_TXBUF
               shared_half_done[refill_half[c]]++;
               refill_half[c] ^= 1U;
#else
               fill_half(c, refill_half[c], &lane_prbs[c]);
               refill_half[c] ^= 1U;
#endif
               lane_done_halves[c]++;
               if (c == 0U && PROG_HALVES != 0U &&
                   (lane_done_halves[0] & (PROG_HALVES - 1U)) == 0U) {
                  put_str("tx h=");
                  put_u32(lane_done_halves[0]);
                  put_str("\r\n");
               }
            }
            all_done = false;
         }
      }
#if SHARED_TXBUF
      while (shared_half_done[shared_refill_half] >= NCH) {
         shared_half_done[shared_refill_half] -= NCH;
         fill_half(0U, shared_refill_half, &shared_prbs);
         shared_refill_half ^= 1U;
      }
#endif
      if (!saw_first && lane_done_halves[0] > 0U) { saw_first = true; diag("wrap1"); }
#if DIAG
      if ((lane_done_halves[0] & 0x1ffU) == 0x1ffU && lane_done_halves[0] != diag_last) {
         diag_last = lane_done_halves[0];
         put_str("h="); put_u32(lane_done_halves[0]); put_str("\r\n");
      }
#endif
      uint32_t now = timer_ticks();
      elapsed += (uint64_t)(uint32_t)(now - last_tick);
      last_tick = now;
      if (elapsed >= tick_limit)
         tx_timeouts = 1U;
   }

   diag("loopexit");
   // Keep the SPORT clocks running ~1 ms past the final word: the FPGA
   // registers data/FS in IO cells, so the last word needs trailing
   // clock edges to flush through the pipeline and latch its done flag.
   // The receiver's report values are latched at done, so the trailing
   // idle clocks are harmless.
   delay_us(1000U);
   for (uint32_t c = 0U; c < NCH; c++) {
      sport_disable(TX_ID[c], SPORT_HALF_A);
      dma_disable(TX_DMA[c]);
      dma_wait_idle(TX_DMA[c]);
   }
   gpio_write(GPIO_DAI1_06, false);

   uint32_t sent = lane_done_halves[0] * HALF_WORDS;
   for (uint32_t c = 1U; c < NCH; c++) {
      uint32_t s = lane_done_halves[c] * HALF_WORDS;
      if (s < sent) sent = s;
   }

   put_str("sport_fpga_rx sent=");
   put_u32(sent);
   put_str(" lanes=");
   put_u32(NCH);
   put_str(" bit_clk=");
   put_u32(SPORT_BIT_CLK_HZ);
   put_str(" agg_rate_bps=");
   put_u32(AGG_RATE_BPS);
   put_str(" tx_timeouts=");
   put_u32(tx_timeouts);
   put_str((sent >= N_WORDS && tx_timeouts == 0U) ? " PASS\r\n" : " FAIL\r\n");

   for (;;) { }
}
