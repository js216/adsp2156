// SPDX-License-Identifier: MIT
// main.c --- IIR accelerator biquad low-pass filter demo for the EV-21569-SOM
// Copyright (c) 2026 Jakob Kastelic

// Generates a two-tone test signal (200 Hz + 4000 Hz at 48 kHz),
// passes it through the hardware IIR accelerator with a 2nd-order
// Butterworth low-pass biquad (cutoff ~1000 Hz), and verifies
// that the high-frequency component is attenuated while the
// low-frequency component passes through.

#include "board.h"
#include "clocks.h"
#include "iir.h"
#include "timer.h"
#include "uart.h"
#include <stdint.h>
#include <stdio.h>

// 2nd-order Butterworth LPF biquad, fc=1000 Hz, fs=48000 Hz.
// Computed via bilinear transform, normalized by a0.
//
// Storage order per biquad (HRM 39-5):
//   Ak0, Ak1, Bk1, Ak2, Bk2, Dk2, Dk1
// where Akx = bx (numerator), Bkx = -ax (negated denominator)
#define NBIQUADS 1U
static const float lpf_biquad[NBIQUADS * IIR_WORDS_PER_BIQUAD] = {
    0.003919f,  // Ak0 = b0
    0.007838f,  // Ak1 = b1
    1.815180f,  // Bk1 = -a1
    0.003919f,  // Ak2 = b2
    -0.831244f, // Bk2 = -a2
    0.0f,       // Dk2 (initial state)
    0.0f,       // Dk1 (initial state)
};

#define WINDOW 256U
#define FS     48000U
#define F_LO   200U
#define F_HI   4000U
#define PI_F   3.14159265f

static float fabsf_(float x)
{
   return x < 0.0f ? -x : x;
}

static float fmodf_(float x, float y)
{
   return x - (float)(int)(x / y) * y;
}

static float sinf_(float x)
{
   x = fmodf_(x, 2.0f * PI_F);
   if (x > PI_F)
      x -= 2.0f * PI_F;
   if (x < -PI_F)
      x += 2.0f * PI_F;
   float num = 16.0f * x * (PI_F - x);
   float den = 5.0f * PI_F * PI_F - 4.0f * x * (PI_F - x);
   if (fabsf_(den) < 1e-12f)
      return 0.0f;
   return num / den;
}

#pragma section("seg_l2_bss", NO_INIT)
static float coefs[NBIQUADS * IIR_WORDS_PER_BIQUAD];

#pragma section("seg_l2_bss", NO_INIT)
static float inbuf[WINDOW];

#pragma section("seg_l2_bss", NO_INIT)
static float outbuf[WINDOW];

static float peak_abs(const float *buf, unsigned n)
{
   float mx = 0.0f;
   for (unsigned i = 0; i < n; i++) {
      float a = fabsf_(buf[i]);
      if (a > mx)
         mx = a;
   }
   return mx;
}

int main(void)
{
   static const struct clocks_cfg clk = BOARD_CLOCKS_CFG;
   clocks_init(&clk);
   uart_init(BOARD_BAUD_DIV);
   timer_init();
   delay_ms(1500);

   printf("\r\niir demo starting\r\n");

   // Copy biquad coefficients + initial state to L2
   for (unsigned i = 0; i < NBIQUADS * IIR_WORDS_PER_BIQUAD; i++)
      coefs[i] = lpf_biquad[i];

   // Generate two-tone input signal
   for (unsigned i = 0; i < WINDOW; i++) {
      float t  = (float)i / (float)FS;
      inbuf[i] = sinf_(2.0f * PI_F * (float)F_LO * t) +
                 sinf_(2.0f * PI_F * (float)F_HI * t);
   }

   float in_peak = peak_abs(inbuf, WINDOW);
   printf("in_peak %x\r\n", *(unsigned *)&in_peak);

   // Run the hardware IIR
   iir_init();
   struct iir_cfg cfg = {
       .coefs     = coefs,
       .input     = inbuf,
       .output    = outbuf,
       .nbiquads  = NBIQUADS,
       .window    = WINDOW,
       .in_count  = WINDOW,
       .out_count = WINDOW,
   };
   uint32_t st = iir_run(&cfg);
   printf("stat %x\r\n", st);

   // Skip initial transient, measure steady state
   unsigned skip = 128;
   float ss_peak = peak_abs(&outbuf[skip], WINDOW - skip);
   printf("ss_peak %x\r\n", *(unsigned *)&ss_peak);

   // Print a few steady-state output IEEE bits
   for (unsigned i = skip; i < WINDOW && i < skip + 8; i++)
      printf("y[%x] %x\r\n", i, *(unsigned *)&outbuf[i]);

   // Output peak should be well below input peak
   if (ss_peak < in_peak * 0.7f && ss_peak > 0.1f)
      printf("PASS iir_attenuation\r\n");
   else
      printf("FAIL iir_attenuation\r\n");

   printf("iir demo done\r\n");
   for (;;)
      delay_ms(1000);
}
