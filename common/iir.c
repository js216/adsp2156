// SPDX-License-Identifier: MIT
// iir.c --- SHARC+ IIR hardware accelerator driver
// Copyright (c) 2026 Jakob Kastelic

// Legacy-mode driver for the IIR0 accelerator. Cascaded biquad
// sections (Transposed Direct Form II) are computed by the
// hardware MAC engine. Coefficients, state, input, and output
// buffers reside in DMA-reachable system memory (L2).

#include "iir.h"
#include "assert.h"
#include "regs.h"
#include <stdint.h>

// TCB field indices for legacy mode (HRM 39-12, Table 39-5).
enum {
   TCB_CHNPTR  = 0,
   TCB_COEFLEN = 1,
   TCB_COEFMOD = 2,
   TCB_COEFIDX = 3,
   TCB_OUTBASE = 4,
   TCB_OUTLEN  = 5,
   TCB_OUTMOD  = 6,
   TCB_OUTIDX  = 7,
   TCB_INBASE  = 8,
   TCB_INLEN   = 9,
   TCB_INMOD   = 10,
   TCB_INIDX   = 11,
   TCB_CTL2    = 12,
   TCB_LEN     = 13,
};

#define IIR_POLL_TIMEOUT 10000000U

static uint32_t to_word_addr(const void *p)
{
   uint32_t byte_addr = (uint32_t)p;
   return byte_addr / 4U;
}

#pragma section("seg_l2_bss", NO_INIT)
static uint32_t iir_tcb[TCB_LEN];

void iir_init(void)
{
   MMR(REG_DPM0_PER_DIS0) &= ~(1U << 1U);
   MMR(REG_IIR0_CTL1) = 0;
   (void)MMR(REG_IIR0_DMASTAT);
   (void)MMR(REG_IIR0_MACSTAT);
}

uint32_t iir_run(const struct iir_cfg *cfg)
{
   ASSERT(cfg->nbiquads >= 1 && cfg->nbiquads <= IIR_MAX_BIQUADS);
   ASSERT(cfg->window >= 1 && cfg->window <= 1024);

   uint32_t coef_addr = to_word_addr(cfg->coefs);
   uint32_t in_addr   = to_word_addr(cfg->input);
   uint32_t out_addr  = to_word_addr(cfg->output);
   uint32_t coef_len  = cfg->nbiquads * IIR_WORDS_PER_BIQUAD;

   uint32_t ctl2 = ((cfg->window - 1U) << POS_IIR_CTL2_WINDOW) |
                   ((cfg->nbiquads - 1U) << POS_IIR_CTL2_BIQUADS);

   // Self-referencing chain pointer for single-channel operation.
   // The HRM says the last channel's TCB should point back to
   // the first -- for single channel, that means self.
   uint32_t tcb_end = to_word_addr(&iir_tcb[TCB_CTL2]);

   iir_tcb[TCB_CHNPTR]  = tcb_end; // self-referencing
   iir_tcb[TCB_COEFLEN] = coef_len;
   iir_tcb[TCB_COEFMOD] = 1;
   iir_tcb[TCB_COEFIDX] = coef_addr;
   iir_tcb[TCB_OUTBASE] = out_addr;
   iir_tcb[TCB_OUTLEN]  = cfg->out_count;
   iir_tcb[TCB_OUTMOD]  = 1;
   iir_tcb[TCB_OUTIDX]  = out_addr;
   iir_tcb[TCB_INBASE]  = in_addr;
   iir_tcb[TCB_INLEN]   = cfg->in_count;
   iir_tcb[TCB_INMOD]   = 1;
   iir_tcb[TCB_INIDX]   = in_addr;
   iir_tcb[TCB_CTL2]    = ctl2;

   MMR(REG_IIR0_CHNPTR) = tcb_end;

   // Enable without SS first, then disable after one pass.
   // CAI=0 means stop after processing all channels once.
   MMR(REG_IIR0_CTL1) = BIT_IIR_CTL1_EN | BIT_IIR_CTL1_DMAEN;

   volatile uint32_t timeout = IIR_POLL_TIMEOUT;
   uint32_t done_mask        = BIT_IIR_DMASTAT_ACDONE | BIT_IIR_DMASTAT_WDONE;
   while (!(MMR(REG_IIR0_DMASTAT) & done_mask)) {
      if (--timeout == 0)
         break;
   }

   uint32_t stat      = MMR(REG_IIR0_DMASTAT);
   MMR(REG_IIR0_CTL1) = 0;
   return stat;
}
