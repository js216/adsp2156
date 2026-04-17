// SPDX-License-Identifier: MIT
// iir.h --- SHARC+ IIR hardware accelerator driver
// Copyright (c) 2026 Jakob Kastelic

// Driver for the IIR0 system accelerator on the ADSP-2156x.
// Supports legacy-mode single-channel biquad filtering with
// 32-bit IEEE floating-point data. The accelerator implements
// cascaded Transposed Direct Form II second-order sections.
//
// Each biquad section uses 7 words of coefficient/state memory:
//   b0, b1, -a1, b2, -a2, d2, d1
// where b* are numerator coefficients, a* are denominator
// coefficients (stored negated), and d* are delay-line state.
// Note: a0 is assumed to be 1.0 (normalized form).

#ifndef IIR_H
#define IIR_H

#include <stdint.h>

// Maximum biquad sections per channel.
#define IIR_MAX_BIQUADS 64U

// Words per biquad section in 32-bit mode (5 coefficients + 2 states).
#define IIR_WORDS_PER_BIQUAD 7U

// Descriptor for a single-channel IIR filtering pass. All
// pointer fields must point into DMA-reachable memory (L2 or
// system-alias L1). Coefficient/state data must be laid out
// as IIR_WORDS_PER_BIQUAD words per section, contiguous.
struct iir_cfg {
   float *coefs;       // coefficient+state buffer, nbiquads*7 words
   float *input;       // input sample buffer
   float *output;      // output sample buffer
   uint32_t nbiquads;  // number of biquad sections (1..64)
   uint32_t window;    // output samples per pass (1..1024)
   uint32_t in_count;  // input buffer length (>= window)
   uint32_t out_count; // output buffer length (>= window)
};

// Initialise the IIR0 accelerator. Call once at startup.
void iir_init(void);

// Execute a single-channel IIR filter pass using legacy mode.
// Blocks (polls) until the accelerator finishes. Returns the
// IIR_DMASTAT register value for diagnostics.
uint32_t iir_run(const struct iir_cfg *cfg);

#endif // IIR_H
