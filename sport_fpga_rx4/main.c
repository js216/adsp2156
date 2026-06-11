// SPDX-License-Identifier: MIT
// main.c --- Four SPORT half-A master TX lanes into an FPGA receiver
// Copyright (c) 2026 Jakob Kastelic
//
// SPORT4/0/5/1 half-A are configured as master TX
// (internal clock, internal frame sync). The DSP uses SCLK0 = 62.5 MHz;
// SPORT CLKDIV=0 -> bit_clk = SCLK0 / (CLKDIV + 1).
// SPORT4A and SPORT0A are routed onto wired DAI pin triples. SPORT4A's
// default PB01/PB02/PB04 path is not active on this bench, so lane 0 uses
// DAI1 PB05/PB07/PB08. SPORT0A uses DAI0 PB05/PB07/PB08.
//
// Sends N_WORDS of an incrementing 32-bit counter. The core polls the
// SPORT TX FIFO status and writes the
// next word set as soon as all lanes can accept data, keeping the
// transmit path bounded by the SPORT bit clock instead of a software
// spin delay.
//
// The default run sends 64 MiB per lane.

#include "board.h"
#include "clocks.h"
#include "regs.h"
#include "sport.h"
#include "timer.h"
#include "uart.h"
#include <stdbool.h>
#include <stdint.h>

#ifndef N_WORDS
#define N_WORDS               16777216U
#endif
#ifndef TX_TIMEOUT_S
#define TX_TIMEOUT_S          30U
#endif
#ifndef TX_SAMPLE_RISING
#define TX_SAMPLE_RISING      1
#endif
#define SPORT_WORD_BITS       32U
#define SPORT_FSDIV           31U
#define SPORT_SCLK_HZ         62500000U
#define SPORT_CLKDIV          0U
#define SPORT_BIT_CLK_HZ      62500000U
#define NCH                   4U
#define AGG_RATE_BPS          (SPORT_BIT_CLK_HZ * NCH)
#define LOCAL_BAUD_DIV        ((SPORT_SCLK_HZ + (BOARD_BAUD / 2U)) / BOARD_BAUD)
#ifndef DIAG_STAGES
#define DIAG_STAGES           0U
#endif

static const enum sport_id TX_ID[NCH] = {
    SPORT_ID_4, SPORT_ID_0, SPORT_ID_5, SPORT_ID_1};

static void put_str(const char *s)
{
   while (*s != '\0')
      uart_putc(*s++);
}

static void diag(const char *s)
{
#if DIAG_STAGES
   put_str(s);
   put_str("\r\n");
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

static bool tx_all_ready(void)
{
   for (uint32_t c = 0U; c < NCH; c++) {
      if (!sport_tx_ready(TX_ID[c], SPORT_HALF_A))
         return false;
   }
   return true;
}

static void tx_write_all(uint32_t *word)
{
   for (uint32_t c = 0U; c < NCH; c++)
      sport_write_raw(TX_ID[c], SPORT_HALF_A, *word);
   (*word)++;
}

static struct sport_dsp_cfg tx_master_cfg = {
    .word_bits     = SPORT_WORD_BITS,
    .clkdiv        = SPORT_CLKDIV,
    .fsdiv         = SPORT_FSDIV,
    .is_tx         = true,
    .internal_clk  = true,
    .internal_fs   = true,
    .late_fs       = true,
    .data_indep_fs = false,
    .sample_rising = (TX_SAMPLE_RISING != 0),
};

int main(void)
{
   static const struct clocks_cfg clk = CLOCKS_CFG_SCLK0_62MHZ;
   clocks_init(&clk);
   uart_init(LOCAL_BAUD_DIV);
   diag("entry");
   while ((MMR(REG_UART0_STAT) & BIT_UART_STAT_THRE) == 0U) {
   }
   diag("after_clocks");
   timer_init();
   diag("after_timer");
   board_som_init(0U);
   diag("after_board");

   put_str("\r\nsport_fpga_rx boot channels=4 sport4a+sport0a+sport5a+sport1a\r\n");

   diag("route4");
   sport_route_tx_to_pins(SPORT_ID_4, 1U, 5U, 7U, 8U);
   diag("route0");
   sport_route_sport0a_to_wired_pins();
   diag("route5");
   sport_route_tx_to_pins(SPORT_ID_5, 1U, 9U, 10U, 11U);
   diag("route1");
   sport_route_tx_to_pins(SPORT_ID_1, 0U, 10U, 12U, 20U);
   diag("route_done");
   for (uint32_t c = 0U; c < NCH; c++) {
      diag("init_lane");
      sport_dsp_serial_init(TX_ID[c], SPORT_HALF_A, &tx_master_cfg);
      sport_clear_errors(TX_ID[c], SPORT_HALF_A);
   }
   diag("init_done");

   uint32_t word = 0U;

   for (uint32_t c = 0U; c < NCH; c++) {
      sport_write_raw(TX_ID[c], SPORT_HALF_A, word);
      sport_write_raw(TX_ID[c], SPORT_HALF_A, word + 1U);
   }
   word = 2U;

   for (uint32_t c = 0U; c < NCH; c++)
      sport_enable(TX_ID[c], SPORT_HALF_A);

   uint32_t sent = 2U;
   uint32_t tx_timeouts = 0U;
   uint32_t deadline = timer_ticks() + (SPORT_SCLK_HZ * TX_TIMEOUT_S);
   while (sent < N_WORDS && tx_timeouts == 0U) {
      if (tx_all_ready()) {
         tx_write_all(&word);
         sent++;
      }
      if ((int32_t)(timer_ticks() - deadline) >= 0)
         tx_timeouts = 1U;
   }

   for (volatile uint32_t k = 0U; k < 1000000U; k++) {
   }

   for (uint32_t c = 0U; c < NCH; c++)
      sport_disable(TX_ID[c], SPORT_HALF_A);

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
   put_str((sent == N_WORDS && tx_timeouts == 0U) ? " PASS\r\n" : " FAIL\r\n");

   for (;;) {
   }
}
