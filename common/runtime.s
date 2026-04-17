// SPDX-License-Identifier: MIT
// runtime.s --- Compiler runtime intrinsics for the SHARC+
// Copyright (c) 2026 Jakob Kastelic

// The SHARC+ C compiler emits calls to these helper functions
// for operations that have no single-instruction implementation
// on the core. Building with -no-std-lib means no runtime
// library is linked, so we provide independent implementations.
//
// Provenance:
//   __lib_fdiv -- RECIPS + Newton-Raphson reciprocal iteration,
//     a textbook algorithm (Cavanagh 1984). The RECIPS instruction
//     and an example NR sequence are documented in the publicly
//     available ADSP-21160 SHARC DSP Instruction Set Reference,
//     Chapter 7, page 7-43.
//   __divrem_u32 -- standard binary long division (shift-subtract),
//     any computer architecture textbook.
//   Calling convention -- determined by inspecting the assembly
//     output of the C compiler (-S flag).
//
// __lib_fdiv      -- IEEE 754 single-precision float division
// __lib_fdiv_simd -- same, for SIMD mode
// __divrem_u32    -- unsigned 32-bit integer divmod
//
// Calling convention (from compiler -S output):
//   __lib_fdiv:    F4 = numerator, F8 = denominator, F0 = result
//   __divrem_u32:  R0 = dividend, R4 = divisor,
//                  R0 = quotient, R1 = remainder
//   Scratch: R0, R1, R2, R4, R8, R12 (and F aliases).
//   All other registers are callee-saved.

.SECTION/CODE/DOUBLEANY seg_swco;

// ================================================================
// __lib_fdiv: float division via RECIPS + Newton-Raphson
// ================================================================
// Called via `call (pc,...)`.
// F4 = numerator (N), F8 = denominator (D). Returns F0 = N/D.
// RECIPS gives an 8-bit seed for 1/D; three NR iterations
// refine to full 32-bit IEEE single precision.
// Scratch registers only: F0, F2, F4, F8, F12.

.GLOBAL __lib_fdiv.;
__lib_fdiv.:
   F0 = RECIPS F8;        // F0 = seed for 1/D
   R2 = 0x40000000;       // F2 = 2.0

   // NR iteration 1
   F12 = F0 * F8;
   F12 = F2 - F12;
   F0 = F0 * F12;

   // NR iteration 2
   F12 = F0 * F8;
   F12 = F2 - F12;
   F0 = F0 * F12;

   // NR iteration 3
   F12 = F0 * F8;
   F12 = F2 - F12;
   F0 = F0 * F12;

   // Result: N * (1/D)
   F0 = F0 * F4;
   RTS;
.__lib_fdiv..end:
   .type __lib_fdiv.,STT_FUNC;

// ================================================================
// __lib_fdiv_simd: SIMD float division
// ================================================================

// SIMD version: must set S2 = 2.0 for PEy as well since
// immediate loads do not mirror to shadow registers.
.GLOBAL __lib_fdiv_simd.;
__lib_fdiv_simd.:
   R2 = 0x40000000;
   S2 = 0x40000000;       // PEy constant
   F0 = RECIPS F8;
   F12 = F0 * F8;
   F12 = F2 - F12;
   F0 = F0 * F12;
   F12 = F0 * F8;
   F12 = F2 - F12;
   F0 = F0 * F12;
   F12 = F0 * F8;
   F12 = F2 - F12;
   F0 = F0 * F12;
   F0 = F0 * F4;
   RTS;
.__lib_fdiv_simd..end:
   .type __lib_fdiv_simd.,STT_FUNC;

// ================================================================
// __divrem_u32: unsigned 32-bit integer divide and remainder
// ================================================================
// Inputs:  R0 = dividend, R4 = divisor
// Outputs: R0 = quotient, R1 = remainder
// Scratch used: R0, R1, R2

// Called via: cjump __divrem_u32. (db);
//             dm(i7,m7)=r2;           // push caller frame
//             dm(i7,m7)=.LCJn-1;      // push return address
// Return via: pop i12, jump (m14,i12)

.GLOBAL __divrem_u32.;
__divrem_u32.:
   R1 = 0;                // remainder
   R8 = 32;               // loop counter (R8 is scratch)
.udiv_top:
      R0 = R0 + R0;       // left-shift dividend, MSB -> AC
      R1 = R1 + R1 + CI;  // shift carry into remainder
      R2 = R1 - R4;       // trial subtract
      IF GE R1 = R2;      // keep if remainder >= divisor
      IF GE R0 = BSET R0 BY 0;  // set quotient bit
      R8 = R8 - 1;
      IF NE JUMP .udiv_top;
   // Return from cjump. CJUMP set I6 = old_I7, and the delay
   // slots pushed old_I6 at DM(0,I6) and return_addr at
   // DM(-1,I6). RFRAME restores I7=I6, I6=DM(0,I6).
   I12 = DM(M7, I6);      // read return addr from *(I6 - 1)
   JUMP (M14, I12) (DB);   // jump to return_addr + 1
      RFRAME;              // I7 = I6; I6 = DM(0, I6)
      NOP;
.__divrem_u32..end:
   .type __divrem_u32.,STT_FUNC;
