// SPDX-License-Identifier: MIT
// assert.c --- Project __assert_fail handler (banner + halt)
// Copyright (c) 2026 Jakob Kastelic

// __assert_fail backs both libsel's standard `assert(e)` macro
// and the project's `ASSERT(cond)` extension. On failure it
// prints a short banner through the UART-backed printf and
// then spins forever so the core does not keep running past
// the failure. The banner only lands if uart_init() has
// already been called; otherwise it is lost and the infinite
// loop still catches the failure as a hang.

#include <assert.h>
#include <stdint.h>
#include <stdio.h>

void __assert_fail(const char *expr, const char *file, int line)
{
   printf("\r\nASSERTION FAILED: %s:%u: %s\r\n", file, (uint32_t)line, expr);
   for (;;) {
      // halt
   }
}
