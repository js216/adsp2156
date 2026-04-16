// SPDX-License-Identifier: MIT
// main.c --- GPIO loopback check using hardware jumpers
// Copyright (c) 2026 Jakob Kastelic
//
// Drives three DAI1 pin buffers as GPIO outputs and reads the
// corresponding RX-side pins every half cycle. With three
// jumpers in place between header P13 pins
//
//      pin 2 <-> pin 10   (DAI1_PB01  -> DAI1_PB05)
//      pin 6 <-> pin 14   (DAI1_PB03  -> DAI1_PB07)
//      pin 8 <-> pin 16   (DAI1_PB04  -> DAI1_PB08)
//
// the three input pins should mirror the three output pins.
// Output is a single status line per half cycle, showing the
// commanded value, what PIN_STAT actually reads back on each
// RX pin, and a per-channel PASS/FAIL.

#include "board.h"
#include "clocks.h"
#include "gpio.h"
#include "timer.h"
#include "uart.h"
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#define PHASE_MS 100U

// Three loopback pairs: tx drives, rx reads back through jumper.
static const enum gpio_pin tx_pins[3] = {
    GPIO_DAI1_01,
    GPIO_DAI1_03,
    GPIO_DAI1_04,
};

static const enum gpio_pin rx_pins[3] = {
    GPIO_DAI1_05,
    GPIO_DAI1_07,
    GPIO_DAI1_08,
};

// Corresponding bit positions in DAI1_PIN_STAT for each RX pin,
// so we can compose a compact per-phase readout.
static const uint32_t rx_stat_bit[3] = {4, 6, 7};

static void drive_all(bool high)
{
   for (uint32_t i = 0; i < 3U; i++) {
      gpio_write(tx_pins[i], high);
   }
}

// Decode the three RX bits out of a DAI1_PIN_STAT value.
static uint32_t rx_triplet(uint32_t stat)
{
   return ((stat >> rx_stat_bit[0]) & 1U) << 2U |
          ((stat >> rx_stat_bit[1]) & 1U) << 1U |
          ((stat >> rx_stat_bit[2]) & 1U) << 0U;
}

int main(void)
{
   static const struct clocks_cfg clk = BOARD_CLOCKS_CFG;
   clocks_init(&clk);
   uart_init(BOARD_BAUD_DIV);
   timer_init();
   printf("\r\ngpio 3-channel loopback starting\r\n");

   for (uint32_t i = 0; i < 3U; i++) {
      gpio_make_output(tx_pins[i]);
      gpio_make_input(rx_pins[i]);
   }

   uint32_t cycle     = 0;
   uint32_t passes[3] = {0, 0, 0};
   uint32_t fails[3]  = {0, 0, 0};

   for (;;) {
      drive_all(true);
      delay_ms(PHASE_MS);
      uint32_t hi = rx_triplet(gpio_read_bank(GPIO_BANK_DAI1));

      drive_all(false);
      delay_ms(PHASE_MS);
      uint32_t lo = rx_triplet(gpio_read_bank(GPIO_BANK_DAI1));

      // For channel c, the RX should be 1 in HIGH phase and
      // 0 in LOW phase. Anything else is a FAIL.
      for (uint32_t c = 0; c < 3U; c++) {
         uint32_t bit_hi = (hi >> (2U - c)) & 1U;
         uint32_t bit_lo = (lo >> (2U - c)) & 1U;
         if (bit_hi == 1U && bit_lo == 0U) {
            passes[c]++;
         } else {
            fails[c]++;
         }
      }

      printf("c %x hi=%x lo=%x  ch1=%x/%x ch2=%x/%x ch3=%x/%x\r\n", cycle, hi,
             lo, passes[0], fails[0], passes[1], fails[1], passes[2], fails[2]);
      cycle++;
   }
}
