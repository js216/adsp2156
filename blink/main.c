// SPDX-License-Identifier: MIT
// main.c --- LED blink demo for the EV-21569-SOM
// Copyright (c) 2026 Jakob Kastelic

// Toggles three LEDs driven by a Microchip MCP23017 GPIO
// expander on TWI2 at 7-bit address 0x21. Port A bits 0..2
// are the LEDs; bit 5 is the active-low UART0_EN line that
// gates UART0 TX to the carrier-board header, left low so
// the UART stays enabled while the blink runs.

#include "board.h"
#include "clocks.h"
#include "mcp23017.h"
#include "pinmux.h"
#include "timer.h"
#include "twi.h"

// 7-bit I2C address of the on-SOM MCP23017. Factory-set by the
// three address strap pins (A2 A1 A0) = 0 0 1.
#define LED_EXPANDER_ADDR 0x21U

// Port A bit assignments on this SOM. Bits 0..2 are the three
// on-board LEDs (LED4, LED6, LED7); bit 5 is UART0_EN (drive
// low to let UART0 TX reach the carrier-board header); bit 7
// is a read-only strap, so we configure it as an input. Bits
// 3, 4, 6 are outputs but unused.
#define LED_BITS         0x07U
#define PORTA_INPUT_MASK 0x80U // bit 7 input, 0..6 output

// MCP23017 port B is unused; mark all pins as inputs.
#define MCP23017_DIR_ALL_INPUT 0xFFU

// TWI2 bus timing: prescale = 12 (SCLK / 12 ~ 10 MHz internal
// timebase), CLKDIV = 50/50 -> SCL ~ 100 kHz.
#define TWI_PRESCALE 12U
#define TWI_CLKDIV   50U

#define BLINK_MS 800U

int main(void)
{
   static const struct clocks_cfg clk = BOARD_CLOCKS_CFG;
   clocks_init(&clk);
   timer_init();

   pinmux_twi2();
   static const struct twi_clk_cfg twi_cfg = {TWI_PRESCALE, TWI_CLKDIV,
                                              TWI_CLKDIV};
   twi_init(&twi_cfg);

   // Configure the MCP23017 once: port A bit 7 is an input,
   // bits 0..6 are outputs, port B is all inputs (unused).
   static const struct mcp23017_target led_a = {LED_EXPANDER_ADDR,
                                                MCP23017_PORT_A};
   static const struct mcp23017_target led_b = {LED_EXPANDER_ADDR,
                                                MCP23017_PORT_B};
   mcp23017_set_direction(&led_a, PORTA_INPUT_MASK);
   mcp23017_set_direction(&led_b, MCP23017_DIR_ALL_INPUT);

   for (;;) {
      mcp23017_write_gpio(&led_a, LED_BITS);
      delay_ms(BLINK_MS);

      mcp23017_write_gpio(&led_a, 0U);
      delay_ms(BLINK_MS);
   }
}
