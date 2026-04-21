// SPDX-License-Identifier: MIT
// assert.c --- __assert_fail handler (banner + halt)
// Copyright (c) 2026 Jakob Kastelic

// libsel's <assert.h> leaves sel_assert_fail undefined by
// design so applications can choose their own failure
// behaviour. This is the adsp2156 project's choice: print a
// short banner through the UART-backed printf and spin
// forever so the core does not keep running past a failed
// assertion. The banner only lands if uart_init() has already
// been called; otherwise it is lost and the infinite loop
// still catches the failure as a hang.

#include <assert.h>
#include <stdint.h>
#include <stdio.h>

void sel_assert_fail(const char *expr, const char *file, int line)
{
   printf("\r\nASSERTION FAILED: %s:%u: %s\r\n", file, (uint32_t)line, expr);
   for (;;) {
      // halt
   }
}
