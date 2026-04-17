// SPDX-License-Identifier: MIT
// fir.h --- SHARC+ FIR hardware accelerator driver
// Copyright (c) 2026 Jakob Kastelic

// Driver for the FIR0 system accelerator on the ADSP-2156x.
// Supports legacy-mode single-channel single-rate filtering
// with 32-bit IEEE floating-point data. The accelerator has
// its own DMA engine that fetches coefficients and input data
// from system memory and writes results back, so all buffers
// must be in DMA-reachable memory (L2 SRAM or the L1 system-
// port alias range).

#ifndef FIR_H
#define FIR_H

#include <stdint.h>

// Maximum tap length supported by the accelerator in a single
// iteration (hardware coefficient memory depth).
#define FIR_MAX_TAPS 1024U

// Descriptor for a single-channel FIR filtering pass. All
// pointer fields must point into DMA-reachable memory (L2 or
// system-alias L1). Coefficients must be stored in time-
// reversed order: h[N-1], h[N-2], ..., h[0].
struct fir_cfg {
   const float *coefs; // coefficient buffer (reversed), N entries
   float *input;       // input sample buffer, ntaps-1+window entries
   float *output;      // output sample buffer, window entries
   uint32_t ntaps;     // filter length (1..1024)
   uint32_t window;    // output samples per pass (1..1023, 0=sample)
   uint32_t in_count;  // total input buffer length (ntaps-1+window)
};

// Initialise the FIR0 accelerator. Call once at startup before
// fir_run(). Clears any stale state from a previous session.
void fir_init(void);

// Execute a single-channel single-rate FIR filter pass using
// legacy mode. Blocks (polls) until the accelerator finishes.
// The output buffer receives `cfg->window` IEEE-float samples.
// Returns the FIR_DMASTAT register value for diagnostics.
uint32_t fir_run(const struct fir_cfg *cfg);

#endif // FIR_H
