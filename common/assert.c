// SPDX-License-Identifier: MIT
// assert.c --- Runtime assertion failure handler
// Copyright (c) 2026 Jakob Kastelic

// Runtime assertion failures print a short banner and then
// spin forever. The banner tries to go out UART0 via the
// existing printf hook, which will only work if the demo
// already called uart_init() at startup; otherwise the
// banner is lost but the infinite loop still catches the
// failure as a hang. Either way, the core does not keep
// running past the failure point.

#include "assert.h"
#include <stdint.h>
#include <stdio.h>

void assert_fail(const char *file, int line, const char *cond)
{
   printf("\r\nASSERTION FAILED: %s:%x: %s\r\n", file, (uint32_t)line, cond);
   for (;;) {
      // halt
   }
}
