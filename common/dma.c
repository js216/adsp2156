// SPDX-License-Identifier: MIT
// dma.c --- SHARC+ DMA channel autobuffer driver
// Copyright (c) 2026 Jakob Kastelic

// The dedicated-DMA-engine (DDE) channels share a uniform
// 0x80-stride register layout (HRM 27-3, Table 27-1). The
// driver only supports the simple continuous AUTO (autobuffer)
// flow mode with 4-byte memory and 4-byte peripheral word
// sizes; the caller hands in a circular memory region and the
// DDE keeps cycling through it forever.

#include "dma.h"
#include "assert.h"
#include "regs.h"
#include <stdint.h>

// Base addresses of the two DDE blocks on ADSP-21569. DDE0
// (channels 0..7) lives at DMA0_BASE; DDE1 (channels 10..17)
// at DMA10_BASE. Channels 8 and 9 (MDMA0) live in a separate
// block further up and are not handled by this driver.
#define DMA0_BASE          0x31022000U // HRM 27-66
#define DMA10_BASE         0x31023000U // HRM 27-66
#define DMA_CHANNEL_STRIDE 0x80U       // HRM 27-3

// Channel-number range bounds for the two DDE blocks.
#define DMA0_LAST_CH   7U
#define DMA10_FIRST_CH 10U
#define DMA10_LAST_CH  17U

// DDE per-channel register offsets (HRM 27-3, Table 27-1).
#define OFF_DMA_DSCPTR_NXT 0x00 // HRM 27-40
#define OFF_DMA_ADDRSTART  0x04 // HRM 27-44
#define OFF_DMA_CFG        0x08 // HRM 27-49
#define OFF_DMA_XCNT       0x0C // HRM 27-58
#define OFF_DMA_XMOD       0x10 // HRM 27-59
#define OFF_DMA_YCNT       0x14 // HRM 27-60
#define OFF_DMA_YMOD       0x18 // HRM 27-61
#define OFF_DMA_DSCPTR_CUR 0x24 // HRM 27-62
#define OFF_DMA_DSCPTR_PRV 0x28 // HRM 27-63
#define OFF_DMA_ADDR_CUR   0x2C // HRM 27-64
#define OFF_DMA_STAT       0x30 // HRM 27-65
#define OFF_DMA_XCNT_CUR   0x34 // HRM 27-67
#define OFF_DMA_YCNT_CUR   0x38 // HRM 27-68

// DMA_CFG bits (HRM 27-50..57, Table 27-21).
#define BIT_DMA_CFG_EN    (1U << 0U)
#define BIT_DMA_CFG_WNR   (1U << 1U) // 0 = TX (mem read), 1 = RX (mem write)
#define POS_DMA_CFG_PSIZE 4U
#define POS_DMA_CFG_MSIZE 8U
#define POS_DMA_CFG_FLOW  12U
#define DMA_PSIZE_4B      2U // 4-byte peripheral word
#define DMA_MSIZE_4B      2U // 4-byte memory word
#define DMA_FLOW_AUTO     1U // autobuffer

// L1 system-port alias range. The SHARC+ core sees the 1.5 MB
// of L1 SRAM through a low byte-address window at L1_INT_BASE;
// every other master (peripheral DMA, MDMA, the second core on
// multi-core parts) sees the same bytes aliased L1_SYS_OFFSET
// higher. L2 and off-chip DDR are already on the shared address
// map and pass through unchanged.
#define L1_INT_BASE   0x00240000U // HRM 2-3
#define L1_INT_END    0x0039FFFFU
#define L1_SYS_OFFSET 0x28000000U // HRM 2-3

// Compute the per-channel MMR base. DMA0..7 live in the first
// DDE block; DMA10..17 live 0x1000 higher in the second block
// (the 0x31022xxx region between them holds DMA8/9 == MDMA0).
static uint32_t dma_base(const enum dma_channel ch)
{
   uint32_t n = (uint32_t)ch;
   ASSERT(n <= DMA0_LAST_CH || (n >= DMA10_FIRST_CH && n <= DMA10_LAST_CH));
   if (n <= DMA0_LAST_CH) {
      return DMA0_BASE + (n * DMA_CHANNEL_STRIDE);
   }
   return DMA10_BASE + ((n - DMA10_FIRST_CH) * DMA_CHANNEL_STRIDE);
}

// Translate a core-side L1 address to the system-fabric alias.
// Addresses outside the L1 window pass through unchanged.
static uint32_t to_dma_addr(const void *buf)
{
   uint32_t a = (uint32_t)buf;
   if (a >= L1_INT_BASE && a <= L1_INT_END) {
      a += L1_SYS_OFFSET;
   }
   return a;
}

void dma_autobuffer_config(const enum dma_channel ch, const struct dma_buf buf,
                           const enum dma_dir dir)
{
   uint32_t base = dma_base(ch);

   // Writing CFG while the channel is running triggers a DMA
   // error; clearing the EN bit first is always safe.
   MMR(base + OFF_DMA_CFG) = 0U;

   MMR(base + OFF_DMA_ADDRSTART)  = to_dma_addr(buf.base);
   MMR(base + OFF_DMA_XCNT)       = buf.word_count;
   MMR(base + OFF_DMA_XMOD)       = 4U;
   MMR(base + OFF_DMA_YCNT)       = 0U;
   MMR(base + OFF_DMA_YMOD)       = 0U;
   MMR(base + OFF_DMA_DSCPTR_NXT) = 0U;

   uint32_t cfg = (DMA_FLOW_AUTO << POS_DMA_CFG_FLOW) |
                  (DMA_MSIZE_4B << POS_DMA_CFG_MSIZE) |
                  (DMA_PSIZE_4B << POS_DMA_CFG_PSIZE);
   if (dir == DMA_DIR_RX_TO_MEM) {
      cfg |= BIT_DMA_CFG_WNR;
   }
   MMR(base + OFF_DMA_CFG) = cfg;
}

void dma_enable(const enum dma_channel ch)
{
   uint32_t base = dma_base(ch);
   MMR(base + OFF_DMA_CFG) |= BIT_DMA_CFG_EN;
}

void dma_disable(const enum dma_channel ch)
{
   uint32_t base = dma_base(ch);
   MMR(base + OFF_DMA_CFG) &= ~BIT_DMA_CFG_EN;
}

uint32_t dma_addr_cur(const enum dma_channel ch)
{
   return MMR(dma_base(ch) + OFF_DMA_ADDR_CUR);
}
