// SPDX-License-Identifier: MIT
// stdio.h --- Freestanding stdio for ADSP-21569
// Copyright (c) 2026 Jakob Kastelic

// This project builds freestanding (-no-std-lib) with no hosted
// C library. This header is the project's stdio.h: it provides
// printf and putchar via the tiny printf implementation in
// printf.c, and the application supplies a putchar definition
// that routes bytes to the UART or wherever else is needed.

#ifndef STDIO_H
#define STDIO_H

#include "printf.h"

#endif // STDIO_H
