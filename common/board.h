// SPDX-License-Identifier: MIT
// board.h --- Board-level constants for the EV-21569-SOM
// Copyright (c) 2026 Jakob Kastelic

#ifndef BOARD_H
#define BOARD_H

#include "clocks.h"
#include <stdint.h>

// ----------------------------------------------------------------------
// Clocks
// ----------------------------------------------------------------------

// CGU0 configuration for the EV-21569-SOM. CLKIN = 25 MHz,
// MSEL = 60, SYSSEL = 4, S0SEL = 4 -- SCLK0 = 93.75 MHz.
// These values match the boot ROM UART-host-mode defaults
// (measured by reading CGU_CTL / CGU_DIV at startup).
// clocks_init() verifies the registers and only reprograms
// them if they differ, so the normal-case fast path is a
// pair of MMR reads and a return.
#define BOARD_CLOCKS_CFG CLOCKS_CFG_DEFAULT

// SCLK0 frequency that BOARD_CLOCKS_CFG produces, used by the
// UART baud-rate divisor and the timer driver. Must match the
// SCLK0 that cfg produces; if you change BOARD_CLOCKS_CFG you
// must update this too.
#define BOARD_SCLK_HZ 93750000U

// ----------------------------------------------------------------------
// UART
// ----------------------------------------------------------------------

// Target serial baud rate (8N1). The divisor is computed at
// compile time from BOARD_SCLK_HZ.
#define BOARD_BAUD     115200U
#define BOARD_BAUD_DIV ((BOARD_SCLK_HZ + (BOARD_BAUD / 2U)) / BOARD_BAUD)

// ----------------------------------------------------------------------
// MCP23017 expander (SOM)
// ----------------------------------------------------------------------

// Bring up TWI2 and program the SOM's MCP23017 expander (I2C
// address 0x21) so the on-SOM IS25LP512M flash is isolated from
// PA_00..PA_05 and UART0 TX stays routed to the carrier-board
// header.  Every firmware example that uses SPI2 (RX or TX) or
// UART0 must call this before touching those peripherals; the
// flash otherwise contends SPI2_MISO and UART0 TX silently
// routes nowhere.
//
// Port A bit assignments per SOM schematic:
//   GPA0..GPA2 LEDs (LED6 / LED7 / LED4, active-high)
//   GPA3       *SPIFLASH_CS_EN   driven HIGH (disable flash CS)
//   GPA4       *SPI2D2_D3_EN     driven HIGH (disable flash D2/D3)
//   GPA5       *UART0_EN         driven LOW  (route UART0 TX)
//   GPA6       *UART0_FLOW_EN    driven HIGH (flow gate unused)
//   GPA7       strap, input
//
// If leds_mask is non-zero, those bits (0..2) are ORed into the
// port A output value.  Pass 0 when LEDs should stay off.
void board_som_init(uint32_t leds_mask);

// Re-write port A with the control bits plus a new leds_mask.
// Cheaper than board_som_init -- skips pinmux / twi_init.  Call
// this from an animation loop.
void board_som_set_leds(uint32_t leds_mask);

#endif // BOARD_H
