// SPDX-License-Identifier: MIT
// main.c --- Three-phase LED blink demo for the EV-21569-SOM
// Copyright (c) 2026 Jakob Kastelic

// Drives the three on-SOM LEDs via the MCP23017 expander on
// TWI2 at 7-bit address 0x21.  Each LED runs at the same
// frequency (50% duty, period PERIOD_MS) but phase-shifted 120
// degrees from the next, synthesising a three-phase blink.
//
// Per SOM schematic:
//   GPA0 = LED6            (active-high)
//   GPA1 = LED7            (active-high)
//   GPA2 = LED4            (active-high)
//   GPA3 = *SPIFLASH_CS_EN (set HIGH so flash stays isolated)
//   GPA4 = *SPI2D2_D3_EN   (set HIGH so flash stays isolated)
//   GPA5 = *UART0_EN       (set LOW  so UART0 TX reaches header)
//   GPA6 = *UART0_FLOW_EN  (set HIGH, flow gate unused)
//   GPA7 = strap input
// Isolation bits and UART routing are handled by board_som_init;
// this file only supplies the LED slot pattern.

#include "board.h"
#include "clocks.h"
#include "timer.h"
#include <stdint.h>

// Full waveform period.  Resolved into 6 equal slots so 50%-duty
// 120-degree phases (LED1 at +T/3, LED2 at +2T/3) land on slot
// boundaries.
#define PERIOD_MS 900U
#define SLOT_MS   (PERIOD_MS / 6U)

// Port-A LED bitmasks for each of the 6 slots.  LED0=GPA0,
// LED1=GPA1, LED2=GPA2.
//
// slot | t / PERIOD | LED0 LED1 LED2 | bits
//   0  | [0,  1/6)  |  on  off  on   | 0b101 = 0x05
//   1  | [1/6,2/6)  |  on  off off   | 0b001 = 0x01
//   2  | [2/6,3/6)  |  on  on  off   | 0b011 = 0x03
//   3  | [3/6,4/6)  |  off on  off   | 0b010 = 0x02
//   4  | [4/6,5/6)  |  off on  on    | 0b110 = 0x06
//   5  | [5/6,1)    |  off off on    | 0b100 = 0x04
#define LED_PHASE_COUNT 6U

static const uint8_t led_phase_table[LED_PHASE_COUNT] = {0x05U, 0x01U, 0x03U,
                                                         0x02U, 0x06U, 0x04U};

int main(void)
{
   static const struct clocks_cfg clk = BOARD_CLOCKS_CFG;
   clocks_init(&clk);
   timer_init();

   board_som_init(0U);

   unsigned slot = 0U;
   for (;;) {
      board_som_set_leds(led_phase_table[slot]);
      delay_ms(SLOT_MS);
      slot = (slot + 1U) % LED_PHASE_COUNT;
   }
}
