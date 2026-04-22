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

   // SPI2 shares its PA0..PA5 pin bus with OSPI0 (the chip's
   // SPI3/OSPI controller). Two things have to be lined up or
   // SPI2 never sees external traffic even though its own
   // registers read back sensible values:
   //
   //   1. SCB5_REMAP selects which block owns the SPI2 / OSPI
   //      memory-mapped flash window. 0 = SPI2. The boot ROM may
   //      leave this pointing at OSPI.
   //
   //   2. OSPI0_CTL.EN, if set, drives the shared PA0..PA5 pin
   //      group even after SCB5_REMAP is cleared. The boot ROM
   //      leaves OSPI0 enabled on parts that booted from OSPI
   //      flash. Its drivers then fight the SPI2 pad drivers and
   //      the slave-mode SPI2 receive path (which depends on
   //      clean SS and SCLK edges) never advances -- TUR latches
   //      from the first edge but SPI_STAT.RFE never clears.
   //      Disable OSPI0 unconditionally before SPI2 comes up.
   if (id == SPI_ID_2) {
      MMR(REG_OSPI0_CTL) &= ~BIT_OSPI_CTL_EN;
      MMR(REG_SCB5_REMAP) = 0U;
   }

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

   // SPI_SLVSEL programming:
   //   - Master role: SSE1 | SSEL1 -- enable PA5 as SEL1 output,
   //     deasserted high.
   //   - Slave role:  0x0000FE00 -- all SSE*=0 (tri-state every
   //     SPI_SEL output) with SSEL*=1 as the deassert default if
   //     any SSE is later enabled. This is CCES's
   //     DEFAULT_SPISLVSEL_OUTPUT in adi_spi_data_2156x.c.
   //
   // Programming SLVSEL unconditionally on both transitions is
   // load-bearing: without the slave-branch write, SLVSEL
   // inherited SSE1=1 from the prior master-mode SELFTEST. That
   // kept the peripheral's SEL1 output driver enabled while the
   // block ran as a slave, so PA5 (which carries both SPI2_SEL1b
   // and SPI2_SSb) was driven from inside the DSP at the same
   // time the external FT4222 tried to pull it low as CS0. The
   // slave shifter saw no clean SS assertion and never promoted a
   // received word from the shift register into RFIFO.
   if (cfg->is_master)
      MMR(base + OFF_SPI_SLVSEL) = BIT_SPI_SLVSEL_SSE1 | BIT_SPI_SLVSEL_SSEL1;
   else
      MMR(base + OFF_SPI_SLVSEL) = SPI_SLVSEL_SLAVE_IDLE;

   // Prepare TXCTL / RXCTL but leave TEN/REN = 0 for now.
   // TTI (transmit transfer initiate) and RTI (receive transfer
   // initiate) are MASTER-ONLY. In slave mode the ADI reference
   // driver clears both (see
   // CCES/SHARC/lib/src/drivers/Source/spi/adi_spi_2156x.c ::
   // EnableSPIChannel, master branch sets TTI/RTI, slave branch
   // sets neither). With TTI/RTI set in slave mode the peripheral
   // never advances the receive state machine when the external
   // master toggles SCLK, so SPI_STAT.RFE stays latched forever
   // even though TUR confirms the clocks are being seen.
   if (cfg->is_master) {
      MMR(base + OFF_SPI_TXCTL) = BIT_SPI_TXCTL_TTI;
      MMR(base + OFF_SPI_RXCTL) = BIT_SPI_RXCTL_RTI;
   } else {
      MMR(base + OFF_SPI_TXCTL) = 0;
      MMR(base + OFF_SPI_RXCTL) = 0;
   }
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
