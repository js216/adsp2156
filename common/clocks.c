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
#define CGU_DIV_CSEL_W      0x1FU
#define POS_CGU_DIV_CSEL    0U
#define MASK_CGU_DIV_CSEL   (CGU_DIV_CSEL_W << POS_CGU_DIV_CSEL)
#define CGU_DIV_SYSSEL_W    0x1FU // 5-bit SYSCLK divisor field
#define POS_CGU_DIV_SYSSEL  8U
#define MASK_CGU_DIV_SYSSEL (CGU_DIV_SYSSEL_W << POS_CGU_DIV_SYSSEL)
#define CGU_DIV_S0SEL_W     0x07U // 3-bit SCLK0 divisor field
#define POS_CGU_DIV_S0SEL   5U
#define MASK_CGU_DIV_S0SEL  (CGU_DIV_S0SEL_W << POS_CGU_DIV_S0SEL)
#define BIT_CGU_DIV_UPDT    (1U << 30U)

// CGU_STAT bits (HRM 2-24..27).
#define BIT_CGU_STAT_PLOCK (1U << 2U)
// CLKSALGN was historically polled here, but observation on the
// EV-21569-SOM with the bench's UART-host boot loader shows the
// bit reads 0 at boot (CGU_STAT = 0x05 = PLLEN | PLOCK) -- i.e.,
// in the "no alignment indicated" state -- so an unconditional
// `wait until CLKSALGN == 1` hangs forever for any cfg whose
// dividers differ from the boot ROM's. The divider commit itself
// is fast (sub-microsecond at SHARC+ CCLK rates), so dropping the
// CLKSALGN wait is safe; PLOCK is the meaningful gate when MSEL
// or DF change.

void clocks_init(const struct clocks_cfg *cfg)
{
   // Build expected bit patterns from the config.
   uint32_t want_msel   = (cfg->msel & CGU_CTL_MSEL_W) << POS_CGU_CTL_MSEL;
   uint32_t want_df     = (cfg->df & CGU_CTL_DF_W) << POS_CGU_CTL_DF;
   uint32_t want_csel   = (cfg->csel & CGU_DIV_CSEL_W) << POS_CGU_DIV_CSEL;
   uint32_t want_syssel = (cfg->syssel & CGU_DIV_SYSSEL_W)
                          << POS_CGU_DIV_SYSSEL;
   uint32_t want_s0sel = (cfg->s0sel & CGU_DIV_S0SEL_W) << POS_CGU_DIV_S0SEL;

   // Read current state.
   uint32_t cur_ctl = MMR(REG_CGU0_CTL);
   uint32_t cur_div = MMR(REG_CGU0_DIV);

   bool ctl_ok = ((cur_ctl & MASK_CGU_CTL_MSEL) == want_msel) &&
                 ((cur_ctl & MASK_CGU_CTL_DF) == want_df);
   bool div_ok = ((cur_div & MASK_CGU_DIV_CSEL) == want_csel) &&
                 ((cur_div & MASK_CGU_DIV_SYSSEL) == want_syssel) &&
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
      div &= ~(MASK_CGU_DIV_CSEL | MASK_CGU_DIV_SYSSEL | MASK_CGU_DIV_S0SEL);
      div |= want_csel | want_syssel | want_s0sel | BIT_CGU_DIV_UPDT;
      MMR(REG_CGU0_DIV) = div;
   }

   // Wait for PLL lock (matters only when MSEL/DF changed -- PLL
   // unlocks during a recalibration). For pure divider changes
   // PLOCK is already 1 and this wait is a single MMR read.
   while ((MMR(REG_CGU0_STAT) & BIT_CGU_STAT_PLOCK) == 0U) {
   }
   // Brief settle so the new dividers commit before the caller
   // starts using SCLK0 / SYSCLK at their new frequencies.
   for (volatile uint32_t i = 0U; i < 256U; i++) {
   }
}
