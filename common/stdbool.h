// SPDX-License-Identifier: MIT
// stdbool.h --- Minimal boolean type
// Copyright (c) 2026 Jakob Kastelic

#ifndef STDBOOL_H
#define STDBOOL_H

// The compiler provides _Bool as a built-in type. We just expose
// the C99 stdbool.h macros on top of it.
#define bool  _Bool
#define true  1
#define false 0

#endif // STDBOOL_H
