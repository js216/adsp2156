// SPDX-License-Identifier: MIT
// clocks.h --- Explicit CGU clock initialization
// Copyright (c) 2026 Jakob Kastelic

// Programs CGU0 to a known state instead of relying on
// whatever the boot ROM left running. The default parameters
// reproduce the boot-ROM UART-host-mode configuration
// (SCLK0 = 93.75 MHz), but the caller can override by passing
// a different struct clocks_cfg.

#ifndef CLOCKS_H
#define CLOCKS_H

#include <stdint.h>

// CGU0 configuration. The PLL equation is:
//   PLLCLK = CLKIN / (df + 1) * msel
//   SYSCLK = PLLCLK / syssel
//   SCLK0  = SYSCLK / s0sel
//   CCLK   = PLLCLK / csel
//   DCLK   = PLLCLK / dsel
//   OCLK   = PLLCLK / osel
struct clocks_cfg {
   uint32_t clkin_hz; // on-board oscillator frequency
   uint32_t msel;     // PLL multiplier (1..127; 0 = 128)
   uint32_t df;       // input divider (0 = /1, 1 = /2)
   uint32_t csel;     // CCLK divider (1..31; 0 = 32)
   uint32_t syssel;   // SYSCLK divider (1..31; 0 = 32)
   uint32_t s0sel;    // SCLK0 divider (1..7; 0 = 8)
   uint32_t s1sel;    // SCLK1 divider (1..7; 0 = 8)
   uint32_t dsel;     // DCLK divider (1..31; 0 = 32)
   uint32_t osel;     // OCLK divider (1..127; 0 = 128)
};

// Default configuration matching what the boot ROM leaves on
// the EV-21569-SOM in UART-host boot mode (measured by reading
// CGU_CTL and CGU_DIV at startup):
//   CLKIN = 25 MHz, MSEL = 60, DF = 0
//   PLLCLK = 1500 MHz, CCLK = 750 MHz, SYSCLK = 375 MHz,
//   SCLK0 = 93.75 MHz.
#define CLOCKS_CFG_DEFAULT                                                     \
   {                                                                           \
       .clkin_hz = 25000000U,                                                  \
       .msel     = 60U,                                                        \
       .df       = 0U,                                                         \
       .csel     = 2U,                                                         \
       .syssel   = 4U,                                                         \
       .s0sel    = 4U,                                                         \
       .s1sel    = 2U,                                                         \
       .dsel     = 3U,                                                         \
       .osel     = 40U,                                                        \
   }

// Program CGU0 to the given configuration and wait for PLL
// lock. The caller is responsible for ensuring BOARD_SCLK_HZ
// matches the SCLK0 that `cfg` produces; the function does
// not compute or return the resulting frequency because doing
// so would pull in a 32-bit divide helper that is not
// available in the -no-std-lib build.
//
//   cfg: pointer to the desired CGU0 settings. Must not be NULL.
void clocks_init(const struct clocks_cfg *cfg);

#endif // CLOCKS_H
