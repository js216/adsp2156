// SPDX-License-Identifier: MIT
// timer.h --- Free-running SCLK0 timer for coarse timing
// Copyright (c) 2026 Jakob Kastelic

// Wraps TIMER0 sub-timer 0 in continuous PWMOUT mode clocked
// from SCLK0 as a 32-bit monotonic counter. At SCLK0 =
// 93.75 MHz the counter wraps every ~45.8 seconds, which is
// more than enough for the throughput sampling intervals we
// care about (a few hundred ms). The pin output is disabled
// so the timer does not accidentally drive a PORT pad.

#ifndef TIMER_H
#define TIMER_H

#include "board.h"
#include <stdint.h>

// SCLK0 rate the timer and delay functions use for scaling.
// Single source of truth: BOARD_SCLK_HZ in board.h.
#define TIMER_SCLK_HZ BOARD_SCLK_HZ

// Start TIMER0 sub-timer 0 counting SCLK0 ticks.
// Safe to call more than once; each call resets the counter.
void timer_init(void);

// Read the current monotonic tick count.
//   returns: 32-bit count of SCLK0 cycles since timer_init().
//            Wraps every ~45.8 s at 93.75 MHz. Use unsigned
//            subtraction for elapsed-time arithmetic.
uint32_t timer_ticks(void);

// Busy-wait approximately `us` microseconds.
//   us: delay in microseconds. Safe up to ~11 000 000 (the
//       us * 375 intermediate must fit in a uint32_t).
void delay_us(const uint32_t us);

// Busy-wait approximately `ms` milliseconds.
//   ms: delay in milliseconds. Safe up to ~11 000.
void delay_ms(const uint32_t ms);

#endif // TIMER_H
