// SPDX-License-Identifier: MIT
// spi.c --- Polled SPI driver for the ADSP-2156x
// Copyright (c) 2026 Jakob Kastelic

#include "spi.h"
#include "regs.h"
#include <stdint.h>

// Base addresses for SPI0..2 (HRM 15-2). Stride is 0x1000.
#define SPI_STRIDE   0x1000U
#define SPI_BASE(id) (REG_SPI0_BASE + ((uint32_t)(id) * SPI_STRIDE))

// Per-instance register accessor.
#define SPIREG(id, off) MMR(SPI_BASE(id) + (off))

// Polling iteration cap to avoid hanging forever on a
// misconfigured or disconnected bus.
#define POLL_LIMIT 500000U

// Write-1-to-clear all bits in ILAT_CLR.
#define ILAT_CLR_ALL 0xFFFFFFFFU

void spi_init(enum spi_id id, const struct spi_cfg *cfg)
{
   uint32_t base = SPI_BASE(id);

   // SPI2 shares its pin bus with OSPI (SPI3); the SCB5 remap
   // register picks which block owns the external signals. The
   // boot ROM on the ADSP-2156x may leave this pointing at OSPI
   // -- in that case SPI2 registers are reachable but CLK/MOSI/
   // SEL1 traffic never reaches the PA0..PA5 pins and the RX
   // FIFO stays empty. Clear it here so SPI2 always wins.
   if (id == SPI_ID_2)
      MMR(REG_SCB5_REMAP) = 0U;

   // Disable the module while configuring.
   MMR(base + OFF_SPI_CTL) = 0;

   // Clock rate: baud = SCLK0 / (clkdiv + 1).
   MMR(base + OFF_SPI_CLK) = cfg->clkdiv;

   // No inter-frame delay.
   MMR(base + OFF_SPI_DLY) = 0;

   // Clear any pending status / interrupt latch bits.
   MMR(base + OFF_SPI_STAT)     = MMR(base + OFF_SPI_STAT);
   MMR(base + OFF_SPI_ILAT_CLR) = ILAT_CLR_ALL;

   // Build the CTL word.
   uint32_t ctl = BIT_SPI_CTL_EN;

   if (cfg->is_master)
      ctl |= BIT_SPI_CTL_MSTR;
   if (cfg->cpol)
      ctl |= BIT_SPI_CTL_CPOL;
   if (cfg->cpha)
      ctl |= BIT_SPI_CTL_CPHA;
   if (cfg->lsb_first)
      ctl |= BIT_SPI_CTL_LSBF;

   ctl |= (cfg->size & 3U) << POS_SPI_CTL_SIZE;
   ctl |= (cfg->miom & 3U) << POS_SPI_CTL_MIOM;

   // Slave role: enable MISO output + PSSE so the slave honors
   // the pin slave-select (SPI_SS) input. Master role: enable
   // ASSEL so the SEL1 pin automatically tracks the transfer;
   // the driver's SLVSEL programming below picks SEL1 as the
   // active line.
   // Diagnostic: also set SOSI, because the original peripheral
   // reset state had bit 22 high and we never verified which
   // bits are hardware-required.
   if (!cfg->is_master)
      ctl |= BIT_SPI_CTL_EMISO;
   else
      ctl |= BIT_SPI_CTL_ASSEL;

   MMR(base + OFF_SPI_CTL) = ctl;

   // Configure slave-select 1 as a software-controlled output
   // (master only). Enable the pin and deassert it (high).
   if (cfg->is_master) {
      MMR(base + OFF_SPI_SLVSEL) = BIT_SPI_SLVSEL_SSE1 | BIT_SPI_SLVSEL_SSEL1;
   }

   // Prepare TXCTL / RXCTL but leave TEN/REN = 0 for now.
   // TTI (transmit transfer initiate) causes the master to
   // start clocking when TEN is set. RTI does the same for
   // receive. TDU = 0 means send last word on underrun.
   MMR(base + OFF_SPI_TXCTL) = BIT_SPI_TXCTL_TTI;
   MMR(base + OFF_SPI_RXCTL) = BIT_SPI_RXCTL_RTI;
}

void spi_tx_enable(enum spi_id id)
{
   SPIREG(id, OFF_SPI_TXCTL) |= BIT_SPI_TXCTL_TEN;
}

void spi_rx_enable(enum spi_id id)
{
   SPIREG(id, OFF_SPI_RXCTL) |= BIT_SPI_RXCTL_REN;
}

// RDR = 1: the SPI peripheral asserts a DMA request whenever
// its RX FIFO is non-empty. Higher RDR threshold values
// (request-on-quarter / half / full) trade request rate for
// worst-case FIFO residency; 1 is the safest default.
#define SPI_RXCTL_RDR_NE (1U << POS_SPI_RXCTL_RDR)
#define SPI_RXCTL_RDR_M  (7U << POS_SPI_RXCTL_RDR)

void spi_rx_dma_enable(enum spi_id id)
{
   uint32_t base             = SPI_BASE(id);
   uint32_t v                = MMR(base + OFF_SPI_RXCTL);
   v                         = (v & ~SPI_RXCTL_RDR_M) | SPI_RXCTL_RDR_NE;
   MMR(base + OFF_SPI_RXCTL) = v;
}
