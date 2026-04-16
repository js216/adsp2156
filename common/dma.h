// SPDX-License-Identifier: MIT
// dma.h --- SHARC+ Dedicated DMA channel driver (autobuffer)
// Copyright (c) 2026 Jakob Kastelic

// Minimal autobuffer-mode DMA driver for the DDE-managed
// channels (DMA0..DMA17, mostly peripheral DMAs for SPORT and
// MDMA). Autobuffer / FLOW=AUTO means the DMA cycles through
// the same memory region forever without any descriptor
// fetches; when XCNT_CUR hits zero the channel reloads XCNT
// and ADDRSTART and keeps running. This is the cheapest way
// to keep a peripheral fed for continuous streaming and the
// mode the sport_dma throughput test needs.

#ifndef DMA_H
#define DMA_H

#include <stdbool.h>
#include <stdint.h>

// Channel IDs. Matches the hardware DMA0..DMA17 numbering
// with the peripheral-DMA gap (DMA8/9 are MDMA, not SPORT).
enum dma_channel {
   DMA_CH_SPORT0_A = 0,
   DMA_CH_SPORT0_B = 1,
   DMA_CH_SPORT1_A = 2,
   DMA_CH_SPORT1_B = 3,
   DMA_CH_SPORT2_A = 4,
   DMA_CH_SPORT2_B = 5,
   DMA_CH_SPORT3_A = 6,
   DMA_CH_SPORT3_B = 7,
   DMA_CH_SPORT4_A = 10,
   DMA_CH_SPORT4_B = 11,
   DMA_CH_SPORT5_A = 12,
   DMA_CH_SPORT5_B = 13,
   DMA_CH_SPORT6_A = 14,
   DMA_CH_SPORT6_B = 15,
   DMA_CH_SPORT7_A = 16,
   DMA_CH_SPORT7_B = 17,
};

// Direction of the channel relative to memory. TX_FROM_MEM
// reads from the memory buffer and hands words to the
// peripheral (typical SPORT half-A transmit); RX_TO_MEM
// receives words from the peripheral and writes them into
// the memory buffer (typical SPORT half-B receive).
enum dma_dir {
   DMA_DIR_TX_FROM_MEM = 0,
   DMA_DIR_RX_TO_MEM   = 1,
};

// Circular memory region descriptor for DMA autobuffer mode.
//   base:       start address of the circular memory region.
//               The driver translates L1 addresses into the
//               system-port alias the DDE fabric requires.
//   word_count: number of 32-bit words in the region.
struct dma_buf {
   const void *base;
   uint32_t word_count;
};

// Configure a channel for 32-bit autobuffer DMA between a
// memory region and the channel's dedicated peripheral bus.
// The channel is left disabled on return; the caller flips
// the enable bit with dma_enable once the peripheral is also
// configured and ready to request data.
//
// ch  : which DMA channel (DMA_CH_SPORTn_A or _B).
// buf : circular memory region (see struct dma_buf).
// dir : which way words flow (TX_FROM_MEM or RX_TO_MEM).
void dma_autobuffer_config(const enum dma_channel ch, const struct dma_buf buf,
                           const enum dma_dir dir);

// Set CFG.EN to start the channel.
//   ch: which DMA channel.
void dma_enable(const enum dma_channel ch);

// Clear CFG.EN to stop the channel.
//   ch: which DMA channel.
void dma_disable(const enum dma_channel ch);

// Read ADDR_CUR (current memory address the channel will
// access next). Advances by MSIZE bytes per transfer and
// wraps back to ADDRSTART when XCNT_CUR reaches zero.
//   ch:      which DMA channel.
//   returns: the byte address currently in ADDR_CUR.
uint32_t dma_addr_cur(const enum dma_channel ch);

#endif // DMA_H
