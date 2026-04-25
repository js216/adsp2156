// SPDX-License-Identifier: MIT
// board_som.c --- EV-21569-SOM MCP23017 expander configuration
// Copyright (c) 2026 Jakob Kastelic

#include "board.h"
#include "mcp23017.h"
#include "pinmux.h"
#include "twi.h"
#include <stdint.h>

// 7-bit I2C address of the on-SOM MCP23017.  Straps (A2 A1 A0) =
// (0 0 1).
#define BOARD_EXPANDER_ADDR 0x21U

// Port A direction: bit 7 is a strap input, bits 0..6 outputs.
#define PORTA_DIR_MASK 0x80U

// Port A default output value.  Per SOM schematic:
//   bit 3 *SPIFLASH_CS_EN  : drive HIGH to gate off flash CS
//   bit 4 *SPI2D2_D3_EN    : drive HIGH to gate off flash D2/D3
//   bit 5 *UART0_EN        : drive LOW  to route UART0 TX
//   bit 6 *UART0_FLOW_EN   : drive HIGH to disable flow gate
//   bit 7                  : input (strap), ignored
#define PORTA_CTRL_MASK 0x58U
#define PORTA_LED_MASK  0x07U

// All 8 lines on port B configured as outputs.
#define PORTB_DIR_ALL_OUT 0xFFU

#define TWI_PRESCALE 12U
#define TWI_CLKDIV   50U

static const struct mcp23017_target s_port_a = {BOARD_EXPANDER_ADDR,
                                                MCP23017_PORT_A};
static const struct mcp23017_target s_port_b = {BOARD_EXPANDER_ADDR,
                                                MCP23017_PORT_B};

void board_som_init(uint32_t leds_mask)
{
   pinmux_twi2();
   static const struct twi_clk_cfg twi_cfg = {TWI_PRESCALE, TWI_CLKDIV,
                                              TWI_CLKDIV};
   twi_init(&twi_cfg);

   (void)mcp23017_set_direction(&s_port_a, PORTA_DIR_MASK);
   (void)mcp23017_set_direction(&s_port_b, PORTB_DIR_ALL_OUT);
   board_som_set_leds(leds_mask);
}

void board_som_set_leds(uint32_t leds_mask)
{
   uint32_t out = PORTA_CTRL_MASK | (leds_mask & PORTA_LED_MASK);
   (void)mcp23017_write_gpio(&s_port_a, out);
}
