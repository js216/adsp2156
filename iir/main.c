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

// Read the raw IEEE-754 bit pattern of a float through a union.
// Direct pointer aliasing violates strict aliasing; the union
// path is the language-defined way to inspect float bits.
static uint32_t float_bits(float f)
{
   union {
      float f;
      uint32_t u;
   } punner;

   punner.f = f;
   return punner.u;
}

// 2nd-order Butterworth LPF biquad, fc=1000 Hz, fs=48000 Hz.
// Computed via bilinear transform, normalized by a0.
//
// Storage order per biquad (HRM 39-5):
//   Ak0, Ak1, Bk1, Ak2, Bk2, Dk2, Dk1
// where Akx = bx (numerator), Bkx = -ax (negated denominator)
#define NBIQUADS 1U
static const float lpf_biquad[NBIQUADS * IIR_WORDS_PER_BIQUAD] = {
    0.003919F,  // Ak0 = b0
    0.007838F,  // Ak1 = b1
    1.815180F,  // Bk1 = -a1
    0.003919F,  // Ak2 = b2
    -0.831244F, // Bk2 = -a2
    0.0F,       // Dk2 (initial state)
    0.0F,       // Dk1 (initial state)
};

#define WINDOW 256U
#define FS     48000U
#define F_LO   200U
#define F_HI   4000U

#define TWO_PI_F   6.28318530F
#define PI_F       3.14159265F
#define BHASKARA_A 16.0F
#define BHASKARA_B 5.0F
#define BHASKARA_C 4.0F
#define EPSILON    1e-12F

#define STARTUP_MS  1500U
#define IDLE_MS     1000U
#define TRANSIENT   128U
#define PRINT_COUNT 8U
#define ATTEN_HI    0.7F
#define ATTEN_LO    0.1F

static float abs_f(float x)
{
   return x < 0.0F ? -x : x;
}

static float mod_f(float x, float y)
{
   return x - ((float)(int)(x / y) * y);
}

static float sin_approx(float x)
{
   x = mod_f(x, TWO_PI_F);
   if (x > PI_F)
      x -= TWO_PI_F;
   if (x < -PI_F)
      x += TWO_PI_F;
   float num = BHASKARA_A * x * (PI_F - x);
   float den = (BHASKARA_B * PI_F * PI_F) - (BHASKARA_C * x * (PI_F - x));
   if (abs_f(den) < EPSILON)
      return 0.0F;
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
   float mx = 0.0F;
   for (unsigned i = 0; i < n; i++) {
      float a = abs_f(buf[i]);
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
   board_som_init(0U);
   delay_ms(STARTUP_MS);

   printf("\r\niir demo starting\r\n");

   // Copy biquad coefficients + initial state to L2
   for (unsigned i = 0; i < NBIQUADS * IIR_WORDS_PER_BIQUAD; i++)
      coefs[i] = lpf_biquad[i];

   // Generate two-tone input signal
   for (unsigned i = 0; i < WINDOW; i++) {
      float t  = (float)i / (float)FS;
      inbuf[i] = sin_approx(TWO_PI_F * (float)F_LO * t) +
                 sin_approx(TWO_PI_F * (float)F_HI * t);
   }

   float in_peak = peak_abs(inbuf, WINDOW);
   printf("in_peak %x\r\n", float_bits(in_peak));

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
   float ss_peak = peak_abs(&outbuf[TRANSIENT], WINDOW - TRANSIENT);
   printf("ss_peak %x\r\n", float_bits(ss_peak));

   // Print a few steady-state output IEEE bits
   for (unsigned i = TRANSIENT; i < WINDOW && i < TRANSIENT + PRINT_COUNT; i++)
      printf("y[%x] %x\r\n", i, float_bits(outbuf[i]));

   // Output peak should be well below input peak
   if (ss_peak < (in_peak * ATTEN_HI) && ss_peak > ATTEN_LO)
      printf("PASS iir_attenuation\r\n");
   else
      printf("FAIL iir_attenuation\r\n");

   printf("iir demo done\r\n");
   for (;;)
      delay_ms(IDLE_MS);
}
