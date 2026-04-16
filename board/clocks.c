// SPDX-License-Identifier: MIT
// clocks.c --- Explicit CGU clock initialization
// Copyright (c) 2026 Jakob Kastelic

// Verifies that CGU0's PLL multiplier and SCLK0-determining
// divisors match the requested configuration. If they already
// match (the normal case after boot-ROM UART-host init), the
// function returns without touching any register. If they
// differ, it reprograms the CGU and waits for PLL relock.
//
// Only the three fields that affect SCLK0 are checked:
//   CGU_CTL.MSEL    (PLL multiplier)
//   CGU_DIV.SYSSEL  (SYSCLK divisor)
//   CGU_DIV.S0SEL   (SCLK0 divisor)
//
// Other divisors (CSEL, DSEL, OSEL, S1SEL) are left at
// whatever the boot ROM or preload code set. Touching them
// could disrupt clocks to peripherals we have no visibility
// into from this simple driver.

#include "clocks.h"
#include "regs.h"
#include <stdbool.h>
#include <stdint.h>

// CGU_CTL field (HRM 2-18..21).
#define CGU_CTL_MSEL_W    0x7FU // 7-bit PLL multiplier field
#define POS_CGU_CTL_MSEL  8U
#define MASK_CGU_CTL_MSEL (CGU_CTL_MSEL_W << POS_CGU_CTL_MSEL)
#define CGU_CTL_DF_W      0x01U // 1-bit divider-by-2 field
#define POS_CGU_CTL_DF    0U
#define MASK_CGU_CTL_DF   (CGU_CTL_DF_W << POS_CGU_CTL_DF)

// CGU_DIV fields we care about (HRM 2-22..23).
#define CGU_DIV_SYSSEL_W    0x1FU // 5-bit SYSCLK divisor field
#define POS_CGU_DIV_SYSSEL  8U
#define MASK_CGU_DIV_SYSSEL (CGU_DIV_SYSSEL_W << POS_CGU_DIV_SYSSEL)
#define CGU_DIV_S0SEL_W     0x07U // 3-bit SCLK0 divisor field
#define POS_CGU_DIV_S0SEL   5U
#define MASK_CGU_DIV_S0SEL  (CGU_DIV_S0SEL_W << POS_CGU_DIV_S0SEL)
#define BIT_CGU_DIV_UPDT    (1U << 30U)

// CGU_STAT bits (HRM 2-24..27).
#define BIT_CGU_STAT_PLOCK    (1U << 2U)
#define BIT_CGU_STAT_CLKSALGN (1U << 3U)

void clocks_init(const struct clocks_cfg *cfg)
{
   // Build expected bit patterns from the config.
   uint32_t want_msel   = (cfg->msel & CGU_CTL_MSEL_W) << POS_CGU_CTL_MSEL;
   uint32_t want_df     = (cfg->df & CGU_CTL_DF_W) << POS_CGU_CTL_DF;
   uint32_t want_syssel = (cfg->syssel & CGU_DIV_SYSSEL_W)
                          << POS_CGU_DIV_SYSSEL;
   uint32_t want_s0sel = (cfg->s0sel & CGU_DIV_S0SEL_W) << POS_CGU_DIV_S0SEL;

   // Read current state.
   uint32_t cur_ctl = MMR(REG_CGU0_CTL);
   uint32_t cur_div = MMR(REG_CGU0_DIV);

   bool ctl_ok = ((cur_ctl & MASK_CGU_CTL_MSEL) == want_msel) &&
                 ((cur_ctl & MASK_CGU_CTL_DF) == want_df);
   bool div_ok = ((cur_div & MASK_CGU_DIV_SYSSEL) == want_syssel) &&
                 ((cur_div & MASK_CGU_DIV_S0SEL) == want_s0sel);

   if (ctl_ok && div_ok) {
      return;
   }

   // Mismatch: fix it. Read-modify-write to preserve all
   // fields we're not responsible for.
   if (!ctl_ok) {
      uint32_t ctl = cur_ctl;
      ctl &= ~(MASK_CGU_CTL_MSEL | MASK_CGU_CTL_DF);
      ctl |= want_msel | want_df;
      MMR(REG_CGU0_CTL) = ctl;
   }

   if (!div_ok) {
      uint32_t div = cur_div;
      div &= ~(MASK_CGU_DIV_SYSSEL | MASK_CGU_DIV_S0SEL);
      div |= want_syssel | want_s0sel | BIT_CGU_DIV_UPDT;
      MMR(REG_CGU0_DIV) = div;
   }

   // Wait for PLL lock and clock alignment.
   while ((MMR(REG_CGU0_STAT) & BIT_CGU_STAT_PLOCK) == 0U) {
   }
   while ((MMR(REG_CGU0_STAT) & BIT_CGU_STAT_CLKSALGN) == 0U) {
   }
}
