// SPDX-License-Identifier: MIT
// timer.c --- Free-running SCLK0 timer
// Copyright (c) 2026 Jakob Kastelic

// TIMER0 has one top-level block and eight sub-timers. Each
// sub-timer has its own CFG/CNT/PER/WID/DLY registers at a
// 0x20 stride inside the block, and its RUN bit is one bit of
// the top-level TIMER0_RUN register. This driver only needs
// sub-timer 0 configured in continuous PWMOUT mode (TMODE=12)
// with SCLK as clock source (CLKSEL=0), period at the maximum
// so the count wraps only at 2^32 SCLK0 cycles, and the pin
// output disabled (OUTDIS=1) so the timer doesn't show up on
// any GPIO pad.

#include "timer.h"
#include "regs.h"
#include <stdint.h>

// TIMER_TMR_CFG bit fields (Table 18-37, HRM 18-44..48).
#define BIT_TMR_CFG_OUTDIS    (1U << 11U) // HRM 18-46 (OUTDIS, bit 11)
#define POS_TMR_CFG_CLKSEL    8U          // HRM 18-46 (CLKSEL, bits 9:8)
#define POS_TMR_CFG_TMODE     0U          // HRM 18-47 (TMODE,  bits 3:0)
#define TMR_TMODE_PWMOUT_CONT 12U         // HRM 18-48 (continuous PWMOUT)
#define TMR_CLKSEL_SCLK       0U          // HRM 18-46 (CLKSEL=0 uses SCLK0)

// Timer register constants.
#define TMR_PERIOD_MAX 0xFFFFFFFFU // full 32-bit wrap
#define TMR_WIDTH_HALF 0x80000000U // 50% duty, irrelevant for counter use

// delay_us scaling: us * SCLK_US_NUMER / SCLK_US_DENOM gives
// ticks at 93.75 MHz (93.75 = 375 / 4).
#define SCLK_US_NUMER 375U
#define SCLK_US_DENOM 4U

#define US_PER_MS 1000U

void timer_init(void)
{
   // Stop sub-timer 0 before reconfiguring it.
   MMR(REG_TIMER0_RUN_CLR) = 1U << 0U;

   // Continuous PWMOUT, SCLK clock source, pin output buffer
   // disabled so we do not inadvertently drive a pad.
   MMR(REG_TIMER0_TMR0_CFG) = BIT_TMR_CFG_OUTDIS |
                              (TMR_CLKSEL_SCLK << POS_TMR_CFG_CLKSEL) |
                              (TMR_TMODE_PWMOUT_CONT << POS_TMR_CFG_TMODE);

   // Full-range period so the counter wraps at 2^32 cycles.
   MMR(REG_TIMER0_TMR0_PER) = TMR_PERIOD_MAX;
   MMR(REG_TIMER0_TMR0_WID) = TMR_WIDTH_HALF;
   MMR(REG_TIMER0_TMR0_DLY) = 0U;

   // Kick sub-timer 0 via the RUN_SET register.
   MMR(REG_TIMER0_RUN_SET) = 1U << 0U;
}

uint32_t timer_ticks(void)
{
   return MMR(REG_TIMER0_TMR0_CNT);
}

void delay_us(const uint32_t us)
{
   // wait_ticks = us * (SCLK_HZ / 1e6) = us * 93.75 = us * 375 / 4.
   // For us up to ~11 million, us*375 stays within 32 bits, which
   // covers any reasonable busy-wait.
   uint32_t wait_ticks = (us * SCLK_US_NUMER) / SCLK_US_DENOM;
   uint32_t start      = timer_ticks();
   while ((timer_ticks() - start) < wait_ticks) {
      // spin
   }
}

void delay_ms(const uint32_t ms)
{
   delay_us(ms * US_PER_MS);
}
