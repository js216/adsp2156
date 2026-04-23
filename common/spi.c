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
   // SPI3/OSPI controller). Three things have to be lined up or
   // SPI2 never sees external traffic cleanly:
   //
   //   1. SCB5_REMAP selects which block owns the SPI2 / OSPI
   //      memory-mapped flash window. 0 = SPI2. The boot ROM may
   //      leave this pointing at OSPI.
   //
   //   2. OSPI0_CTL.EN, if set, drives the shared PA0..PA5 pin
   //      group. The boot ROM leaves OSPI0 enabled on parts that
   //      booted from OSPI flash.
   //
   //   3. OSPI0_CTL.DACEN (Direct Access Controller Enable, bit
   //      7) and the BAUD field stay set in the boot-ROM residual
   //      configuration even after EN is cleared: a register read
   //      shows OSPI0_CTL = 0x80780080 on our board right after
   //      boot (IDLE=1, BAUD=0xf, DACEN=1, EN=0). In that state
   //      OSPI's direct-access controller can still source onto
   //      the shared pin group during a spurious AHB access --
   //      the OSPI block's DAC engine is triggered by the first
   //      read to its 256 MiB AHB window and keeps drivers live
   //      until explicitly quiesced. Zero the whole control
   //      register so every OSPI-driven path is off, not just EN.
   //
   // Under slave dual/quad RX this matters because PA0 (D1) is a
   // pure external input -- any residual OSPI DAC drive on PA0
   // would fight the host.
   if (id == SPI_ID_2) {
      MMR(REG_OSPI0_CTL)  = 0U;
      MMR(REG_SCB5_REMAP) = 0U;
   }

   // Disable the module while configuring.
   MMR(base + OFF_SPI_CTL) = 0;

   // Clock rate: baud = SCLK0 / (clkdiv + 1).
   MMR(base + OFF_SPI_CLK) = cfg->clkdiv;

   // No inter-frame delay.
   MMR(base + OFF_SPI_DLY) = 0;

   // Clear the transmit / receive word counters (TWC, TWCR, RWC,
   // RWCR). When TWCEN / RWCEN is asserted the peripheral decrements
   // the counter for every word transferred and stops the transfer
   // at zero; even with TWCEN / RWCEN clear in this driver, the
   // counters hold whatever stale value the previous bring-up
   // sequence left behind. In particular, the boot-path SELFTEST
   // runs SPI2 as a master clocking 4 words before this function
   // is reconfigured into a slave -- leaving counter state that
   // would kick in if a later reconfig accidentally (or
   // deliberately) enables the word-count gate. Zero them
   // unconditionally so every bring-up starts from a clean slate.
   MMR(base + OFF_SPI_TWC)  = 0;
   MMR(base + OFF_SPI_TWCR) = 0;
   MMR(base + OFF_SPI_RWC)  = 0;
   MMR(base + OFF_SPI_RWCR) = 0;

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

   // Slave role: enable MISO output only in single-lane mode.
   // EMISO gates the peripheral's PA0 (SPI2_MISO / D1) output
   // driver. In single-lane slave that is correct -- the slave
   // sources MISO data from its TX shift register. In dual and
   // quad slave mode PA0 is a *receive* lane (D1) driven by the
   // external master; enabling EMISO there creates contention
   // between the peripheral's TX shifter and the incoming data.
   // The symptom on hardware is lane-D1 bits reading stuck at 1
   // starting after the first few bytes, once the TX-side logic
   // kicks in and begins sourcing onto PA0. Matches HRM 15-64:
   // "EMISO is valid only for MISO mode (single-lane slave)."
   // Master role: enable ASSEL so the SEL1 pin automatically
   // tracks the transfer; the driver's SLVSEL programming below
   // picks SEL1 as the active line.
   if (!cfg->is_master) {
      if (cfg->miom == SPI_MIO_SINGLE)
         ctl |= BIT_SPI_CTL_EMISO;
   } else {
      ctl |= BIT_SPI_CTL_ASSEL;
   }

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

// Bounded-wait quiescence + disable.  The slave path is the
// caller we care about: after a reconfigure request arrives over
// UART the external master is (almost certainly) already idle,
// so SPIF is set and the loop exits in zero iterations.  A stuck
// peripheral is bounded at roughly a millisecond of busy-wait.
#define SPI_QUIESCE_LIMIT 100000U

void spi_disable(enum spi_id id)
{
   uint32_t base = SPI_BASE(id);
   // Already off -- nothing to do.  (spi_init is called before
   // the CTL register is known-good on the first boot call.)
   if ((MMR(base + OFF_SPI_CTL) & BIT_SPI_CTL_EN) == 0U) {
      MMR(base + OFF_SPI_CTL)  = 0U;
      MMR(base + OFF_SPI_STAT) = MMR(base + OFF_SPI_STAT);
      return;
   }
   // Wait for the module to finish any in-flight word.  SPIF = 1
   // means "no transfer currently active" (HRM 15-71).  RFE check
   // catches the narrow window where SPIF has set but one last
   // word is still queued in the receive FIFO.
   for (uint32_t i = 0; i < SPI_QUIESCE_LIMIT; i++) {
      uint32_t s = MMR(base + OFF_SPI_STAT);
      if ((s & BIT_SPI_STAT_SPIF) && (s & BIT_SPI_STAT_RFE))
         break;
   }
   MMR(base + OFF_SPI_CTL) = 0U;
   // Clear all W1C status latches so spi_init starts clean.
   MMR(base + OFF_SPI_STAT) = MMR(base + OFF_SPI_STAT);
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
