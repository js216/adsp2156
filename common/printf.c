// SPDX-License-Identifier: MIT
// printf.c --- Tiny formatted output
// Copyright (c) 2026 Jakob Kastelic

// Supports %s, %c, %x and %0Nx. Hex avoids any divide/modulo work,
// so no compiler runtime division helper is needed. Output is pushed
// byte-by-byte through `putchar()`, which the application supplies
// (see printf.h).
//
// Every walk over a byte pointer uses the post-increment idiom
// `char c = *p++;` so the compiler emits the post-modify byte load
// form of the SHARC+ data memory access.

#include "printf.h"
#include <stdarg.h>
#include <stdint.h>

static void emit_hex(uint32_t value, int min_width, char pad)
{
   // Count significant digits MSB-first (always at least one).
   int digits = 1;
   uint32_t v = value >> 4;
   while (v != 0U) {
      digits++;
      v >>= 4;
   }

   // Emit padding ahead of the digits.
   int width   = (min_width > digits) ? min_width : digits;
   int leading = width - digits;
   while (leading > 0) {
      putchar(pad);
      leading--;
   }

   // Emit the digits high to low. No buffer, no reverse walk.
   int shift = (digits - 1) * 4;
   while (shift >= 0) {
      uint32_t nibble = (value >> shift) & 0xFU;
      putchar((char)((nibble < 10U) ? ('0' + nibble) : ('a' + (nibble - 10U))));
      shift -= 4;
   }
}

static void emit_str(const char *s)
{
   if (s == 0) {
      s = "(null)";
   }
   for (;;) {
      char c = *s++;
      if (c == '\0') {
         break;
      }
      putchar(c);
   }
}

int printf(const char *fmt, ...)
{
   va_list ap;
   va_start(ap, fmt);

   for (;;) {
      char c = *fmt++;
      if (c == '\0') {
         break;
      }
      if (c != '%') {
         putchar(c);
         continue;
      }

      // Percent specifier. Optional zero-pad and width digit.
      char pad  = ' ';
      int width = 0;
      c         = *fmt++;
      if (c == '0') {
         pad = '0';
         c   = *fmt++;
      }
      while (c >= '0' && c <= '9') {
         width = (width << 3) + (width << 1) + (c - '0'); // x10
         c     = *fmt++;
      }

      // `c` now holds the conversion character.
      switch (c) {
         case 's': emit_str(va_arg(ap, const char *)); break;
         case 'c': putchar((char)va_arg(ap, int)); break;
         case 'x': emit_hex(va_arg(ap, uint32_t), width, pad); break;
         case '%': putchar('%'); break;
         case '\0':
            // Trailing '%' at end of format string; bail out.
            va_end(ap);
            return 0;
         default:
            putchar('%');
            putchar(c);
            break;
      }
   }

   va_end(ap);
   return 0;
}
