// SPDX-License-Identifier: MIT
// fir.c --- SHARC+ FIR hardware accelerator driver
// Copyright (c) 2026 Jakob Kastelic

// Legacy-mode driver for the FIR0 accelerator. The accelerator
// has its own chain-pointer DMA that loads a 13-word Transfer
// Control Block (TCB) from system memory, fetches coefficients
// and input data into internal SRAM, runs the 4-MAC compute
// engine, and writes the results back to system memory.

#include "fir.h"
#include "assert.h"
#include "regs.h"
#include <stdint.h>

// TCB field indices for legacy mode (HRM 38-14, Table 38-6).
// The chain-pointer DMA reads 13 words from the address in
// FIR_CHNPTR downward, mapping each word to the corresponding
// accelerator register.
enum {
   TCB_CHNPTR  = 0,
   TCB_COEFCNT = 1,
   TCB_COEFMOD = 2,
   TCB_COEFIDX = 3,
   TCB_OUTBASE = 4,
   TCB_OUTCNT  = 5,
   TCB_OUTMOD  = 6,
   TCB_OUTIDX  = 7,
   TCB_INBASE  = 8,
   TCB_INCNT   = 9,
   TCB_INMOD   = 10,
   TCB_INIDX   = 11,
   TCB_CTL2    = 12,
   TCB_LEN     = 13,
};

// Poll iteration limit; prevents an infinite hang if the
// accelerator never completes (hardware fault, bad TCB, etc.).
#define FIR_POLL_TIMEOUT 10000000U

// Convert a byte address to the word-aligned format that every
// FIR address register expects (byte address >> 2).
static uint32_t to_word_addr(const void *p)
{
   uint32_t byte_addr = (uint32_t)p;
   return byte_addr / 4U;
}

// The TCB must live in DMA-reachable memory. L2 is always on
// the system fabric; seg_l2_bss avoids boot-time init costs.
#pragma section("seg_l2_bss", NO_INIT)
static uint32_t fir_tcb[TCB_LEN];

void fir_init(void)
{
   // Ensure the FIR0 peripheral clock is enabled (bit 0 of
   // DPM0_PER_DIS0; 0 = enabled, 1 = disabled)
   MMR(REG_DPM0_PER_DIS0) &= ~(1U << 0U);

   // Disable the accelerator and clear any pending state
   MMR(REG_FIR0_CTL1) = 0;
   // Read DMASTAT and MACSTAT to clear sticky bits
   (void)MMR(REG_FIR0_DMASTAT);
   (void)MMR(REG_FIR0_MACSTAT);
}

uint32_t fir_run(const struct fir_cfg *cfg)
{
   ASSERT(cfg->ntaps >= 1 && cfg->ntaps <= FIR_MAX_TAPS);
   ASSERT(cfg->window >= 1 && cfg->window <= 1023);
   ASSERT(cfg->in_count >= cfg->ntaps - 1 + cfg->window);

   uint32_t coef_addr = to_word_addr(cfg->coefs);
   uint32_t in_addr   = to_word_addr(cfg->input);
   uint32_t out_addr  = to_word_addr(cfg->output);

   // Channel control: tap length and window size
   uint32_t ctl2 = ((cfg->ntaps - 1U) << POS_FIR_CTL2_TAPLEN) |
                   (cfg->window << POS_FIR_CTL2_WINDOW);

   fir_tcb[TCB_CHNPTR]  = 0; // no chain (single shot)
   fir_tcb[TCB_COEFCNT] = cfg->ntaps;
   fir_tcb[TCB_COEFMOD] = 1; // stride 1 word
   fir_tcb[TCB_COEFIDX] = coef_addr;
   fir_tcb[TCB_OUTBASE] = out_addr;
   fir_tcb[TCB_OUTCNT]  = cfg->window;
   fir_tcb[TCB_OUTMOD]  = 1;
   fir_tcb[TCB_OUTIDX]  = out_addr;
   fir_tcb[TCB_INBASE]  = in_addr;
   fir_tcb[TCB_INCNT]   = cfg->in_count;
   fir_tcb[TCB_INMOD]   = 1;
   fir_tcb[TCB_INIDX]   = in_addr;
   fir_tcb[TCB_CTL2]    = ctl2;

   // CHNPTR points at the last TCB word; also word-addressed
   MMR(REG_FIR0_CHNPTR) = to_word_addr(&fir_tcb[TCB_CTL2]);

   // Enable the accelerator: writing CTL1 with EN=1 and DMAEN=1
   // triggers the chain-pointer DMA to start loading the TCB.
   MMR(REG_FIR0_CTL1) =
       BIT_FIR_CTL1_EN | BIT_FIR_CTL1_DMAEN | BIT_FIR_CTL1_BURSTEN;

   // Poll DMASTAT until all-channels-done (with timeout)
   volatile uint32_t timeout = FIR_POLL_TIMEOUT;
   while (!(MMR(REG_FIR0_DMASTAT) &
            (BIT_FIR_DMASTAT_ACDONE | BIT_FIR_DMASTAT_WDONE))) {
      if (--timeout == 0)
         break;
   }

   uint32_t stat = MMR(REG_FIR0_DMASTAT);

   // Disable the accelerator
   MMR(REG_FIR0_CTL1) = 0;
   return stat;
}
