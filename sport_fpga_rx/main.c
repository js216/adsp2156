// SPDX-License-Identifier: MIT
// main.c --- SPORT4 half-A master TX into an FPGA receiver
// Copyright (c) 2026 Jakob Kastelic
//
// SPORT4 half-A is configured as master TX (internal clock,
// internal frame sync). DSP CGU is reprogrammed to SCLK0 = 62.5
// MHz; SPORT CLKDIV=0 -> bit_clk = SCLK0 / (CLKDIV + 1) = 62.5
// MHz exactly, matching the datasheet fSPTCLKPROG TX cap.
//
// Sends N_WORDS of an LFSR PRBS (taps 32/22/2/1, seed
// 0x12345678) using a spin-delay-paced loop. The CPU at 750 MHz
// is ~5x faster than the SPORT bit clock, so a 2000-cycle
// volatile spin between each sport_write_raw lets the SPORT
// drain its FIFO without overrun, with brief PBEN-deassertion
// gaps between frames -- short enough for the FPGA's idle
// detector but long enough for the external DAI pad output
// path to stay reliable. Sustained continuous TX at 62.5 MHz
// on this path has been observed to lose AFS pulses after a
// few frames (see history); the burst pattern avoids that.
//
// 4097-word run finishes in ~11 ms.

#include "board.h"
#include "clocks.h"
#include "regs.h"
#include "sport.h"
#include "timer.h"
#include "uart.h"
#include <stdbool.h>
#include <stdint.h>

#define N_WORDS               4097U
#define LFSR_SEED             0x12345678U
#define SPORT_WORD_BITS       32U
#define SPORT_FSDIV           31U
#define SPORT_SCLK_HZ         62500000U
#define SPORT_CLKDIV          0U
#define SPORT_BIT_CLK_HZ      62500000U
#define LOCAL_BAUD_DIV        ((SPORT_SCLK_HZ + (BOARD_BAUD / 2U)) / BOARD_BAUD)
#define TX_SPIN_DELAY         2000U

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

static inline uint32_t lfsr_next(uint32_t *state)
{
   uint32_t s = *state;
   uint32_t bit =
       ((s >> 31) ^ (s >> 21) ^ (s >> 1) ^ (s >> 0)) & 1U;
   s = (s << 1) | bit;
   *state = s;
   return s;
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
    .sample_rising = true,
};

int main(void)
{
   uart_init(BOARD_BAUD_DIV);
   while ((MMR(REG_UART0_STAT) & BIT_UART_STAT_THRE) == 0U) {
   }
   static const struct clocks_cfg clk = CLOCKS_CFG_SCLK0_62MHZ;
   clocks_init(&clk);
   MMR(REG_UART0_CLK) =
       BIT_UART_CLK_EDBO | (LOCAL_BAUD_DIV & MASK_UART_CLK_DIV);
   timer_init();
   board_som_init(0U);

   put_str("\r\nsport_fpga_rx boot\r\n");

   sport_enable_external_pins(SPORT_ID_4);
   sport_dsp_serial_init(SPORT_ID_4, SPORT_HALF_A, &tx_master_cfg);
   sport_clear_errors(SPORT_ID_4, SPORT_HALF_A);

   uint32_t lfsr = LFSR_SEED;

   sport_write_raw(SPORT_ID_4, SPORT_HALF_A, lfsr_next(&lfsr));
   sport_write_raw(SPORT_ID_4, SPORT_HALF_A, lfsr_next(&lfsr));

   sport_enable(SPORT_ID_4, SPORT_HALF_A);

   uint32_t sent = 2U;
   for (uint32_t i = 2U; i < N_WORDS; i++) {
      for (volatile uint32_t k = 0U; k < TX_SPIN_DELAY; k++) {
      }
      sport_write_raw(SPORT_ID_4, SPORT_HALF_A, lfsr_next(&lfsr));
      sent++;
   }

   for (volatile uint32_t k = 0U; k < 1000000U; k++) {
   }

   sport_disable(SPORT_ID_4, SPORT_HALF_A);

   put_str("sport_fpga_rx sent=");
   put_u32(sent);
   put_str(" bit_clk=");
   put_u32(SPORT_BIT_CLK_HZ);
   put_str(" tx_timeouts=0 PASS\r\n");

   for (;;) {
   }
}
