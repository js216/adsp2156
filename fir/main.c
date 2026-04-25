// SPDX-License-Identifier: MIT
// main.c --- FIR accelerator low-pass filter demo for the EV-21569-SOM
// Copyright (c) 2026 Jakob Kastelic

// Generates a two-tone test signal (200 Hz + 4000 Hz at 48 kHz),
// passes it through the hardware FIR accelerator with a 17-tap
// low-pass filter (cutoff ~1000 Hz), and verifies that the high-
// frequency component is attenuated while the low-frequency
// component passes through. Prints raw IEEE-754 hex values of
// key measurements over UART.

#include "board.h"
#include "clocks.h"
#include "fir.h"
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

// Precomputed 17-tap LPF at fc=1000 Hz, fs=48000 Hz, Hamming
// window. Symmetric; already in the reversed order the FIR
// accelerator expects (h[N-1] first), which for a symmetric
// filter is the same.
#define NTAPS 17U
static const float lpf_coefs[NTAPS] = {
    -0.001152F, 0.001291F, 0.008489F, 0.022854F, 0.045573F,  0.074711F,
    0.105547F,  0.131387F, 0.144800F, 0.131387F, 0.105547F,  0.074711F,
    0.045573F,  0.022854F, 0.008489F, 0.001291F, -0.001152F,
};

#define WINDOW 256U
#define NBUF   (NTAPS - 1U + WINDOW)

#define FS   48000U
#define F_LO 200U
#define F_HI 4000U

#define TWO_PI_F   6.28318530F
#define PI_F       3.14159265F
#define BHASKARA_A 16.0F
#define BHASKARA_B 5.0F
#define BHASKARA_C 4.0F
#define EPSILON    1e-12F

#define STARTUP_MS     1500U
#define IDLE_MS        1000U
#define TRANSIENT_MULT 2U
#define PRINT_COUNT    8U
#define ATTEN_HI       0.7F
#define ATTEN_LO       0.1F

static float abs_f(float x)
{
   return x < 0.0F ? -x : x;
}

static float mod_f(float x, float y)
{
   return x - ((float)(int)(x / y) * y);
}

// Bhaskara I sine approximation, ~0.2% max error
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
static float coefs[NTAPS];

#pragma section("seg_l2_bss", NO_INIT)
static float inbuf[NBUF];

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

   printf("\r\nfir demo starting\r\n");

   // Copy precomputed LPF coefficients into DMA-reachable L2
   for (unsigned i = 0; i < NTAPS; i++)
      coefs[i] = lpf_coefs[i];

   // Build input: zero-padded history + two-tone signal
   for (unsigned i = 0; i < NTAPS - 1; i++)
      inbuf[i] = 0.0F;
   for (unsigned i = 0; i < WINDOW; i++) {
      float t              = (float)i / (float)FS;
      inbuf[NTAPS - 1 + i] = sin_approx(TWO_PI_F * (float)F_LO * t) +
                             sin_approx(TWO_PI_F * (float)F_HI * t);
   }

   float in_peak = peak_abs(&inbuf[NTAPS - 1], WINDOW);
   printf("in_peak %x\r\n", float_bits(in_peak));

   // Run the hardware FIR
   fir_init();
   struct fir_cfg cfg = {
       .coefs    = coefs,
       .input    = inbuf,
       .output   = outbuf,
       .ntaps    = NTAPS,
       .window   = WINDOW,
       .in_count = NBUF,
   };
   uint32_t st = fir_run(&cfg);
   printf("stat %x\r\n", st);

   // Skip the initial transient, measure steady state
   unsigned skip = NTAPS * TRANSIENT_MULT;
   float ss_peak = peak_abs(&outbuf[skip], WINDOW - skip);
   printf("ss_peak %x\r\n", float_bits(ss_peak));

   // Print a few steady-state output samples
   for (unsigned i = skip; i < WINDOW && i < skip + PRINT_COUNT; i++)
      printf("y[%x] %x\r\n", i, float_bits(outbuf[i]));

   // The output peak should be well below the input peak
   // (roughly half, since only one of two tones passes).
   if (ss_peak < (in_peak * ATTEN_HI) && ss_peak > ATTEN_LO)
      printf("PASS lpf_attenuation\r\n");
   else
      printf("FAIL lpf_attenuation\r\n");

   printf("fir demo done\r\n");
   for (;;)
      delay_ms(IDLE_MS);
}
