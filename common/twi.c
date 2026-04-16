// SPDX-License-Identifier: MIT
// twi.c --- Polled TWI2 master-mode driver
// Copyright (c) 2026 Jakob Kastelic

// Just enough TWI2 to push a few bytes to the MCP23017 port
// expander on the EV-21569-SOM. Master writes only, no DMA, no
// interrupts. Every TWI register on the ADSP-2156x is 16-bit
// wide on the APB bus; 32-bit accesses are silently dropped by
// the peripheral bridge, so this driver uses MMR16 throughout.

#include "twi.h"
#include "assert.h"
#include "regs.h"
#include <stdint.h>

#define TWI_ADDR7_MASK 0x7FU
#define TWI_BYTE_MASK  0xFFU

void twi_init(const struct twi_clk_cfg *cfg)
{
   // Disable, flush both FIFOs, then reprogram. The flush pulse
   // (set FLUSH bits, then clear them) clears any state the boot
   // ROM or a previous program may have left in the TX/RX FIFOs.
   MMR16(REG_TWI2_CTL) = 0U;

   MMR16(REG_TWI2_FIFOCTL) =
       (uint16_t)(BIT_TWI_FIFOCTL_TXFLUSH | BIT_TWI_FIFOCTL_RXFLUSH);
   MMR16(REG_TWI2_FIFOCTL) = 0U;

   // Two-step CTL programming: prescale (no EN) first, then EN.
   MMR16(REG_TWI2_CTL)    = (uint16_t)(cfg->prescale & BIT_TWI_CTL_PRESCALE_M);
   MMR16(REG_TWI2_CLKDIV) = (uint16_t)((cfg->clkhi << POS_TWI_CLKDIV_CLKHI) |
                                       (cfg->clklo << POS_TWI_CLKDIV_CLKLO));
   MMR16(REG_TWI2_CTL) =
       (uint16_t)(BIT_TWI_CTL_EN | (cfg->prescale & BIT_TWI_CTL_PRESCALE_M));
}

int twi_write2(uint32_t addr7, uint32_t reg, uint32_t val)
{
   // Clear IMSK so no stale interrupt handler fires during the
   // polled wait below.
   ASSERT(addr7 <= TWI_ADDR7_MASK);
   MMR16(REG_TWI2_IMSK) = 0U;

   // Flush both FIFOs before each transaction.
   MMR16(REG_TWI2_FIFOCTL) =
       (uint16_t)(BIT_TWI_FIFOCTL_TXFLUSH | BIT_TWI_FIFOCTL_RXFLUSH);
   MMR16(REG_TWI2_FIFOCTL) = 0U;

   MMR16(REG_TWI2_MSTRADDR) = (uint16_t)(addr7 & TWI_ADDR7_MASK);
   MMR16(REG_TWI2_ISTAT)    = (uint16_t)(BIT_TWI_ISTAT_TXSERV |
                                      BIT_TWI_ISTAT_MERR | BIT_TWI_ISTAT_MCOMP);

   // Load first byte; hardware pops it when MSTRCTL.EN is set.
   MMR16(REG_TWI2_TXDATA8) = (uint16_t)(reg & TWI_BYTE_MASK);

   // Start master TX with DCNT=2. Hardware auto-stops at zero.
   MMR16(REG_TWI2_MSTRCTL) =
       (uint16_t)(BIT_TWI_MSTRCTL_EN | (2U << POS_TWI_MSTRCTL_DCNT));

   // Wait for TXSERV: FIFO drained, ready for the second byte.
   while ((MMR16(REG_TWI2_ISTAT) & BIT_TWI_ISTAT_TXSERV) == 0U) {
      if (MMR16(REG_TWI2_ISTAT) & BIT_TWI_ISTAT_MERR) {
         MMR16(REG_TWI2_MSTRCTL) = 0U;
         return -1;
      }
   }
   MMR16(REG_TWI2_ISTAT)   = (uint16_t)BIT_TWI_ISTAT_TXSERV;
   MMR16(REG_TWI2_TXDATA8) = (uint16_t)(val & TWI_BYTE_MASK);

   // Wait for MCOMP: master finished the whole transfer + STOP.
   while ((MMR16(REG_TWI2_ISTAT) & BIT_TWI_ISTAT_MCOMP) == 0U) {
      if (MMR16(REG_TWI2_ISTAT) & BIT_TWI_ISTAT_MERR) {
         MMR16(REG_TWI2_MSTRCTL) = 0U;
         return -1;
      }
   }
   MMR16(REG_TWI2_ISTAT)   = (uint16_t)BIT_TWI_ISTAT_MCOMP;
   MMR16(REG_TWI2_MSTRCTL) = 0U;
   return 0;
}
