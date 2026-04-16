// SPDX-License-Identifier: MIT
// stdarg.h --- Minimal variadic-argument support
// Copyright (c) 2026 Jakob Kastelic

// Variadic argument area lives on the (word-addressed) stack as a
// run of 32-bit slots, one per argument. We model va_list as a
// pointer to that slot run and step it one slot at a time. Anything
// larger than 4 bytes per slot is not supported (the demos only
// pass 32-bit values through %x and friends).

#ifndef STDARG_H
#define STDARG_H

#include <stdint.h>

typedef uint32_t *va_list;

// __builtin_va_start receives the slot size of the LAST named arg
// in 32-bit-word units (= 1 for any 4-byte arg) because the
// underlying I register is word-tagged. va_arg then steps the
// pointer one word per argument.
#define _VA_SLOTS(t) (((sizeof(t) + 3U) >> 2) ? ((sizeof(t) + 3U) >> 2) : 1)

#define va_start(ap, v)                                                        \
   ((ap) = (va_list)__builtin_va_start((void *)&(v), _VA_SLOTS(v)))

#define va_arg(ap, t) (*(t *)(ap++))

#define va_end(ap) ((void)((ap) = 0))

#endif // STDARG_H
