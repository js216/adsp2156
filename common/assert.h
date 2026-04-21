// SPDX-License-Identifier: MIT
// assert.h --- Project assertion extensions over libsel's <assert.h>
// Copyright (c) 2026 Jakob Kastelic

// libsel provides the standard C `assert(e)` macro and the
// underlying `__assert_fail(expr, file, line)` hook. This
// header adds two project extensions on top:
//
//   STATIC_ASSERT(cond, msg): compile-time check via a
//   negative-sized typedef; the message is forced into the
//   identifier so the diagnostic carries it.
//
//   ASSERT(cond): runtime check. On failure, calls
//   __assert_fail, which common/assert.c implements to print
//   a file:line:condition banner through the project's printf
//   and then spin forever with the core halted.

#ifndef ASSERT_H
#define ASSERT_H

// Pull in libsel's standard assert() macro and declare
// __assert_fail.
#include <assert.h>

#define STATIC_ASSERT(cond, msg)                                               \
   typedef char _static_assert_##msg[(cond) ? 1 : -1]

#define ASSERT(cond)                                                           \
   do {                                                                        \
      if (!(cond)) {                                                           \
         __assert_fail(#cond, __FILE__, __LINE__);                             \
      }                                                                        \
   } while (0)

#endif // ASSERT_H
