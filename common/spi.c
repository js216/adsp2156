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

   ctl |= (uint32_t)cfg->size << POS_SPI_CTL_SIZE;
   ctl |= (uint32_t)cfg->miom << POS_SPI_CTL_MIOM;

   // In slave mode enable MISO output so the slave can drive
   // data back to the master.
   if (!cfg->is_master)
      ctl |= BIT_SPI_CTL_EMISO;

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

void spi_tx_disable(enum spi_id id)
{
   SPIREG(id, OFF_SPI_TXCTL) &= ~BIT_SPI_TXCTL_TEN;
}

void spi_rx_disable(enum spi_id id)
{
   SPIREG(id, OFF_SPI_RXCTL) &= ~BIT_SPI_RXCTL_REN;
}

void spi_select(enum spi_id id)
{
   // Drive SSEL1 low (assert).
   SPIREG(id, OFF_SPI_SLVSEL) &= ~BIT_SPI_SLVSEL_SSEL1;
}

void spi_deselect(enum spi_id id)
{
   // Drive SSEL1 high (deassert).
   SPIREG(id, OFF_SPI_SLVSEL) |= BIT_SPI_SLVSEL_SSEL1;
}

void spi_write(enum spi_id id, uint32_t word)
{
   uint32_t base = SPI_BASE(id);
   for (uint32_t i = 0; i < POLL_LIMIT; i++) {
      if (!(MMR(base + OFF_SPI_STAT) & BIT_SPI_STAT_TFF)) {
         MMR(base + OFF_SPI_TFIFO) = word;
         return;
      }
   }
}

uint32_t spi_read(enum spi_id id)
{
   uint32_t base = SPI_BASE(id);
   for (uint32_t i = 0; i < POLL_LIMIT; i++) {
      if (!(MMR(base + OFF_SPI_STAT) & BIT_SPI_STAT_RFE))
         return MMR(base + OFF_SPI_RFIFO);
   }
   return SPI_READ_TIMEOUT;
}

int spi_xfer(enum spi_id id, const uint32_t *tx, uint32_t *rx, uint32_t n)
{
   uint32_t base = SPI_BASE(id);

   for (uint32_t i = 0; i < n; i++) {
      // Wait for TX FIFO room.
      uint32_t tries = POLL_LIMIT;
      while (MMR(base + OFF_SPI_STAT) & BIT_SPI_STAT_TFF) {
         if (--tries == 0)
            return -1;
      }
      MMR(base + OFF_SPI_TFIFO) = tx ? tx[i] : 0;

      // Wait for RX data.
      tries = POLL_LIMIT;
      while (MMR(base + OFF_SPI_STAT) & BIT_SPI_STAT_RFE) {
         if (--tries == 0)
            return -1;
      }
      uint32_t got = MMR(base + OFF_SPI_RFIFO);
      if (rx)
         rx[i] = got;
   }

   // Check for errors.
   uint32_t stat = MMR(base + OFF_SPI_STAT);
   if (stat & (BIT_SPI_STAT_ROR | BIT_SPI_STAT_TUR | BIT_SPI_STAT_TC))
      return -1;

   return 0;
}

void spi_disable(enum spi_id id)
{
   uint32_t base             = SPI_BASE(id);
   MMR(base + OFF_SPI_TXCTL) = 0;
   MMR(base + OFF_SPI_RXCTL) = 0;
   MMR(base + OFF_SPI_CTL)   = 0;
}
