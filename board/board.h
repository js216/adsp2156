// SPDX-License-Identifier: MIT
// board.h --- Board-level constants for the EV-21569-SOM
// Copyright (c) 2026 Jakob Kastelic

#ifndef BOARD_H
#define BOARD_H

#include "clocks.h"

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

#endif // BOARD_H
