// SPDX-License-Identifier: MIT
// assert.h --- Minimal compile-time and runtime assertion macros
// Copyright (c) 2026 Jakob Kastelic

// Two-level assertion support:
//
//   STATIC_ASSERT(cond, msg): evaluated at compile time via a
//   negative-sized array trick. The message is forced into the
//   identifier name so diagnostic output carries it.
//
//   ASSERT(cond):  runtime check. On failure, prints a
//   file:line:condition banner to UART (via printf, so the UART
//   driver must be running) and drops into an infinite loop
//   with the core halted. Prefer this over silently ignoring a
//   bad argument -- the caller learns about the bug at the
//   first failing call instead of watching a later stage do
//   something incomprehensible.
//
// The runtime assertion calls into a tiny helper, assert_fail,
// defined in assert.c. Applications that do not use it do not
// pay any code-size cost because nothing pulls assert.c in.

#ifndef ASSERT_H
#define ASSERT_H

// Build-time check. Produces a zero-sized array declaration
// when the condition holds, and a negative-sized one (hard
// compiler error) when it does not. The message is part of
// the declared name so it shows up in the error banner.
#define STATIC_ASSERT(cond, msg)                                               \
   typedef char _static_assert_##msg[(cond) ? 1 : -1]

void assert_fail(const char *file, int line, const char *cond);

#define ASSERT(cond)                                                           \
   do {                                                                        \
      if (!(cond)) {                                                           \
         assert_fail(__FILE__, __LINE__, #cond);                               \
      }                                                                        \
   } while (0)

#endif // ASSERT_H
