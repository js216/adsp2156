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
#include "regs.h"
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>

// Base addresses of the DDE blocks on ADSP-21569. DDE0
// (channels 0..7) lives at DMA0_BASE; DDE1 (channels 10..17)
// at DMA10_BASE. Channels 8/9 are MDMA0 (unsupported here).
// Channels 22..27 are SPI0/1/2 TX/RX peripheral-DMA lines
// in a separate block at DMA22_BASE.
#define DMA0_BASE          0x31022000U // HRM 27-66
#define DMA10_BASE         0x31023000U // HRM 27-66
#define DMA22_BASE         0x3102D000U // cdef ADSP_2156x_HPC.h
#define DMA_CHANNEL_STRIDE 0x80U       // HRM 27-3

// Channel-number range bounds for the DDE blocks.
#define DMA0_LAST_CH   7U
#define DMA10_FIRST_CH 10U
#define DMA10_LAST_CH  17U
#define DMA22_FIRST_CH 22U
#define DMA22_LAST_CH  27U

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
#define DMA_FLOW_STOP     0U // one-shot: disable at end of count
#define DMA_FLOW_AUTO     1U // autobuffer
#define DMA_FLOW_DSCL     4U // descriptor-list

// NDSIZE field bits 18:16, NDSIZE=4 => 5 descriptor elements
// fetched (NXT, ADDRSTART, CFG, XCNT, XMOD).
#define POS_DMA_CFG_NDSIZE 16U
#define DMA_NDSIZE_5       4U

// INT field bits 21:20.  Value 1 = assert interrupt (and latch
// DMA_STAT.IRQDONE) when XCNT_CUR reaches 0 at end of work unit.
#define POS_DMA_CFG_INT  20U
#define DMA_INT_ON_XCNT0 1U

// DMA_STAT bits (HRM 27-66, Table 27-25).
#define BIT_DMA_STAT_IRQDONE (1U << 0U)

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
   assert(n <= DMA0_LAST_CH || (n >= DMA10_FIRST_CH && n <= DMA10_LAST_CH) ||
          (n >= DMA22_FIRST_CH && n <= DMA22_LAST_CH));
   if (n <= DMA0_LAST_CH) {
      return DMA0_BASE + (n * DMA_CHANNEL_STRIDE);
   }
   if (n <= DMA10_LAST_CH) {
      return DMA10_BASE + ((n - DMA10_FIRST_CH) * DMA_CHANNEL_STRIDE);
   }
   return DMA22_BASE + ((n - DMA22_FIRST_CH) * DMA_CHANNEL_STRIDE);
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

uint32_t dma_xcnt_cur(const enum dma_channel ch)
{
   return MMR(dma_base(ch) + OFF_DMA_XCNT_CUR);
}

void dma_oneshot_config(const enum dma_channel ch, const struct dma_buf buf,
                        const enum dma_dir dir)
{
   uint32_t base = dma_base(ch);

   MMR(base + OFF_DMA_CFG) = 0U;

   MMR(base + OFF_DMA_ADDRSTART)  = to_dma_addr(buf.base);
   MMR(base + OFF_DMA_XCNT)       = buf.word_count;
   MMR(base + OFF_DMA_XMOD)       = 4U;
   MMR(base + OFF_DMA_YCNT)       = 0U;
   MMR(base + OFF_DMA_YMOD)       = 0U;
   MMR(base + OFF_DMA_DSCPTR_NXT) = 0U;

   uint32_t cfg = (DMA_FLOW_STOP << POS_DMA_CFG_FLOW) |
                  (DMA_MSIZE_4B << POS_DMA_CFG_MSIZE) |
                  (DMA_PSIZE_4B << POS_DMA_CFG_PSIZE);
   if (dir == DMA_DIR_RX_TO_MEM) {
      cfg |= BIT_DMA_CFG_WNR;
   }
   MMR(base + OFF_DMA_CFG) = cfg;
}

bool dma_done(const enum dma_channel ch)
{
   return MMR(dma_base(ch) + OFF_DMA_XCNT_CUR) == 0U;
}

void dma_pingpong_rx_config(const enum dma_channel ch, const void *buf_a,
                            const void *buf_b, uint32_t half_words,
                            struct dma_dscl desc[2])
{
   uint32_t base = dma_base(ch);

   // Each descriptor is identical except for the buffer pointer and
   // its next-pointer, which chain A -> B -> A forever.
   uint32_t cfg = (DMA_NDSIZE_5 << POS_DMA_CFG_NDSIZE) |
                  (DMA_FLOW_DSCL << POS_DMA_CFG_FLOW) |
                  (DMA_INT_ON_XCNT0 << POS_DMA_CFG_INT) |
                  (DMA_MSIZE_4B << POS_DMA_CFG_MSIZE) |
                  (DMA_PSIZE_4B << POS_DMA_CFG_PSIZE) | BIT_DMA_CFG_WNR |
                  BIT_DMA_CFG_EN;

   desc[0].next      = to_dma_addr(&desc[1]);
   desc[0].addrstart = to_dma_addr(buf_a);
   desc[0].cfg       = cfg;
   desc[0].xcnt      = half_words;
   desc[0].xmod      = 4U;

   desc[1].next      = to_dma_addr(&desc[0]);
   desc[1].addrstart = to_dma_addr(buf_b);
   desc[1].cfg       = cfg;
   desc[1].xcnt      = half_words;
   desc[1].xmod      = 4U;

   // Disable, then start via the DSCPTR_NXT + CFG.EN path.  Writing
   // CFG with FLOW=DSCL and NDSIZE=4 causes the DDE to fetch the
   // five elements of descriptor A before beginning the transfer.
   MMR(base + OFF_DMA_CFG)        = 0U;
   MMR(base + OFF_DMA_STAT)       = BIT_DMA_STAT_IRQDONE; // W1C stale latch
   MMR(base + OFF_DMA_DSCPTR_NXT) = to_dma_addr(&desc[0]);
   MMR(base + OFF_DMA_YCNT)       = 0U;
   MMR(base + OFF_DMA_YMOD)       = 0U;
   MMR(base + OFF_DMA_CFG)        = cfg;
}

uint32_t dma_stat_raw(const enum dma_channel ch)
{
   return MMR(dma_base(ch) + OFF_DMA_STAT);
}

bool dma_wrap_check(const enum dma_channel ch)
{
   uint32_t addr = dma_base(ch) + OFF_DMA_STAT;
   uint32_t stat = MMR(addr);
   if (!(stat & BIT_DMA_STAT_IRQDONE))
      return false;
   MMR(addr) = BIT_DMA_STAT_IRQDONE; // W1C
   return true;
}
