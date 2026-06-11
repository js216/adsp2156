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

// SCLK0 = 62.5 MHz configuration. Used by SPORT4 missions that
// need to hit the datasheet bit-clock cap on the
// internal-clock-receive path (fSPTCLKEXT RX = 62.5 MHz) and the
// internal-clock-transmit path (fSPTCLKPROG TX = 62.5 MHz) with
// CLKDIV=0:
//   CLKIN = 25 MHz, MSEL = 60, DF = 0
//   PLLCLK = 1500 MHz, CCLK = 750 MHz, SYSCLK = 375 MHz (unchanged
//   from default), SCLK0 = 62.5 MHz, SCLK1 = 187.5 MHz (unchanged).
// Only S0SEL differs from CLOCKS_CFG_DEFAULT (4 -> 6) so SYSCLK
// and every other domain stay at their boot-ROM values, minimising
// glitches at reprogram time.
// HRM 23-3: SPORT_ACLK = SCLK0 / (CLKDIV + 1). With CLKDIV=0,
// bit_clk = SCLK0 = 62.5 MHz exactly.
// HRM 23-3 constraint fSYSCLK = N * fSCLK0, N in {2,4,6}: 375/62.5 = 6. OK.
#define CLOCKS_CFG_SCLK0_62MHZ                                                 \
   {                                                                           \
       .clkin_hz = 25000000U,                                                  \
       .msel     = 60U,                                                        \
       .df       = 0U,                                                         \
       .csel     = 2U,                                                         \
       .syssel   = 4U,                                                         \
       .s0sel    = 6U,                                                         \
       .s1sel    = 2U,                                                         \
       .dsel     = 3U,                                                         \
       .osel     = 40U,                                                        \
   }

// SCLK0 = 60 MHz configuration for SPORT TX margin testing while meeting
// a 60 Mbps/lane requirement with CLKDIV=0:
//   CLKIN = 25 MHz, MSEL = 72, DF = 0
//   PLLCLK = 1800 MHz, CCLK = 600 MHz, SYSCLK = 360 MHz,
//   SCLK0 = 60 MHz, and 360/60 = 6.
#define CLOCKS_CFG_SCLK0_60MHZ                                                 \
   {                                                                           \
       .clkin_hz = 25000000U,                                                  \
       .msel     = 72U,                                                        \
       .df       = 0U,                                                         \
       .csel     = 3U,                                                         \
       .syssel   = 5U,                                                         \
       .s0sel    = 6U,                                                         \
       .s1sel    = 2U,                                                         \
       .dsel     = 3U,                                                         \
       .osel     = 40U,                                                        \
   }

// SCLK0 = 60.833333 MHz configuration, the smallest integer-MSEL step above
// 60 MHz that preserves SYSCLK/SCLK0 = 6 for SPORT CLKDIV=0:
//   CLKIN = 25 MHz, MSEL = 73, DF = 0
//   PLLCLK = 1825 MHz, CCLK = 608.333 MHz, SYSCLK = 365 MHz,
//   SCLK0 = 60.833333 MHz.
#define CLOCKS_CFG_SCLK0_60833333HZ                                            \
   {                                                                           \
       .clkin_hz = 25000000U,                                                  \
       .msel     = 73U,                                                        \
       .df       = 0U,                                                         \
       .csel     = 3U,                                                         \
       .syssel   = 5U,                                                         \
       .s0sel    = 6U,                                                         \
       .s1sel    = 2U,                                                         \
       .dsel     = 3U,                                                         \
       .osel     = 40U,                                                        \
   }

// SCLK0 = 59.375 MHz configuration: MSEL=57, 5%% below the 62.5 MHz
// datasheet SPORT cap, requested by jk 2026-06-11 after a single-word
// D->F corruption at 4 GiB at full rate. Same divisors as the 62MHZ
// cfg so every ratio condition is preserved:
//   CLKIN = 25 MHz, MSEL = 57, DF = 0
//   PLLCLK = 1425 MHz (in 1.20..2.00 GHz, Table 20), CCLK = 712.5 MHz
//   SYSCLK = 356.25 MHz (= CCLK/2), SCLK0 = 59.375 MHz (N = 6).
#define CLOCKS_CFG_SCLK0_59MHZ                                                 \
   {                                                                           \
       .clkin_hz = 25000000U,                                                  \
       .msel     = 57U,                                                        \
       .df       = 0U,                                                         \
       .csel     = 2U,                                                         \
       .syssel   = 4U,                                                         \
       .s0sel    = 6U,                                                         \
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
