// SPDX-License-Identifier: MIT
// stdint.h --- Minimal fixed-width integer types
// Copyright (c) 2026 Jakob Kastelic

// SHARC+ ADSP-21569 with -char-size-8 (PRM 1-35, Table 1-8):
//   char  = 8  bits
//   short = 16 bits
//   int   = 32 bits
//   long  = 32 bits
// Sized to that ABI; not portable to other targets.

#ifndef STDINT_H
#define STDINT_H

typedef signed char int8_t;
typedef unsigned char uint8_t;
typedef signed short int16_t;
typedef unsigned short uint16_t;
typedef signed int int32_t;
typedef unsigned int uint32_t;
typedef signed long long int64_t;
typedef unsigned long long uint64_t;

typedef int32_t intptr_t;
typedef uint32_t uintptr_t;

#endif // STDINT_H
