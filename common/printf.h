// SPDX-License-Identifier: MIT
// printf.h --- Tiny formatted output API
// Copyright (c) 2026 Jakob Kastelic

// printf is character-output agnostic: it does not include any
// driver header itself. The application must provide an
// implementation of `putchar(int c)` that pushes one byte to
// wherever the output needs to go.

#ifndef PRINTF_H
#define PRINTF_H

// Write a single character. Returns the character cast to int,
// or a negative value on error. The application must provide
// this function; the printf implementation only calls it.
int putchar(const int c);

// Minimal printf supporting only %s, %c, %x and %0Nx. Returns
// the number of characters written, or a negative value on
// error.
int printf(const char *fmt, ...);

#endif // PRINTF_H
