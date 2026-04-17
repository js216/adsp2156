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

// Precomputed 17-tap LPF at fc=1000 Hz, fs=48000 Hz, Hamming
// window. Symmetric; already in the reversed order the FIR
// accelerator expects (h[N-1] first), which for a symmetric
// filter is the same.
#define NTAPS 17U
static const float lpf_coefs[NTAPS] = {
    -0.001152f, 0.001291f, 0.008489f, 0.022854f, 0.045573f,  0.074711f,
    0.105547f,  0.131387f, 0.144800f, 0.131387f, 0.105547f,  0.074711f,
    0.045573f,  0.022854f, 0.008489f, 0.001291f, -0.001152f,
};

#define WINDOW 256U
#define NBUF   (NTAPS - 1U + WINDOW)

#define FS   48000U
#define F_LO 200U
#define F_HI 4000U
#define PI_F 3.14159265f

static float fabsf_(float x)
{
   return x < 0.0f ? -x : x;
}

static float fmodf_(float x, float y)
{
   return x - (float)(int)(x / y) * y;
}

// Bhaskara I sine approximation, ~0.2% max error
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
static float coefs[NTAPS];

#pragma section("seg_l2_bss", NO_INIT)
static float inbuf[NBUF];

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

   printf("\r\nfir demo starting\r\n");

   // Copy precomputed LPF coefficients into DMA-reachable L2
   for (unsigned i = 0; i < NTAPS; i++)
      coefs[i] = lpf_coefs[i];

   // Build input: zero-padded history + two-tone signal
   for (unsigned i = 0; i < NTAPS - 1; i++)
      inbuf[i] = 0.0f;
   for (unsigned i = 0; i < WINDOW; i++) {
      float t              = (float)i / (float)FS;
      inbuf[NTAPS - 1 + i] = sinf_(2.0f * PI_F * (float)F_LO * t) +
                             sinf_(2.0f * PI_F * (float)F_HI * t);
   }

   float in_peak = peak_abs(&inbuf[NTAPS - 1], WINDOW);
   printf("in_peak %x\r\n", *(unsigned *)&in_peak);

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

   // Skip the initial transient, measure steady state
   unsigned skip = NTAPS * 2;
   float ss_peak = peak_abs(&outbuf[skip], WINDOW - skip);
   printf("ss_peak %x\r\n", *(unsigned *)&ss_peak);

   // Print a few steady-state output samples
   for (unsigned i = skip; i < WINDOW && i < skip + 8; i++)
      printf("y[%x] %x\r\n", i, *(unsigned *)&outbuf[i]);

   // The output peak should be well below the input peak
   // (roughly half, since only one of two tones passes).
   if (ss_peak < in_peak * 0.7f && ss_peak > 0.1f)
      printf("PASS lpf_attenuation\r\n");
   else
      printf("FAIL lpf_attenuation\r\n");

   printf("fir demo done\r\n");
   for (;;)
      delay_ms(1000);
}
