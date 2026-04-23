// SPDX-License-Identifier: MIT
// pinmux.c --- Pin-multiplexing helpers for the EV-21569-SOM
// Copyright (c) 2026 Jakob Kastelic

#include "pinmux.h"
#include "regs.h"
#include <stdint.h>

// UART0 is always PA6 (TX) / PA7 (RX), alternate function "b"
// = mux value 1 on every ADSP-21569 variant (HRM 12-61).
// PORTA_FER bits select the pin; PORTA_MUX packs two bits per
// pin starting at bit 0 for PA0.
#define PA_UART0_TX_FER_BIT (1U << 6U)
#define PA_UART0_RX_FER_BIT (1U << 7U)
#define PA_UART0_TX_MUX_POS 12U
#define PA_UART0_RX_MUX_POS 14U

// TWI2 is always PA14 (SCL) / PA15 (SDA), alternate function
// "a" = mux value 0 (HRM 12-61).
#define PA_TWI2_SCL_FER_BIT (1U << 14U)
#define PA_TWI2_SDA_FER_BIT (1U << 15U)
#define PA_TWI2_SCL_MUX_POS 28U
#define PA_TWI2_SDA_MUX_POS 30U

void pinmux_uart0(void)
{
   uint32_t mux = MMR(REG_PORTA_MUX);
   mux &= ~((3U << PA_UART0_TX_MUX_POS) | (3U << PA_UART0_RX_MUX_POS));
   mux |= ((1U << PA_UART0_TX_MUX_POS) | (1U << PA_UART0_RX_MUX_POS));
   MMR(REG_PORTA_MUX) = mux;
   MMR(REG_PORTA_FER) |= (PA_UART0_TX_FER_BIT | PA_UART0_RX_FER_BIT);
}

void pinmux_twi2(void)
{
   uint32_t mux = MMR(REG_PORTA_MUX);
   mux &= ~((3U << PA_TWI2_SCL_MUX_POS) | (3U << PA_TWI2_SDA_MUX_POS));
   MMR(REG_PORTA_MUX) = mux;
   MMR(REG_PORTA_FER) |= (PA_TWI2_SCL_FER_BIT | PA_TWI2_SDA_FER_BIT);
}

// All six SPI2 signals (MISO/MOSI/D2/D3/CLK/SEL1|SS) are routed
// to alt "b" (mux=1) on PA0..PA5 per the ADSP-21569 datasheet
// pin-multiplexing table and HRM 12-61 ("01 = first alternate
// peripheral option"). PA5 in particular exposes both SPI2_SEL1b
// (master output) and SPI2_SSb (slave input) on the SAME alt
// slot; the SPI block chooses input vs output internally based
// on CTL.MSTR. PORT_MUX is only 2 bits wide per pin (HRM 19-28),
// so any claim that SPI2_SSb lives at a different mux value is
// wrong -- datasheet pin lists that separate "SEL1b" and "SSb"
// are listing two peripheral-internal uses of the same alt slot,
// not two distinct mux values. Earlier code programmed PA5 to
// mux=3 in slave mode, which is actually SMC0_D05 (static
// memory controller data line) and leaves the SPI block's SS
// input unconnected -- SCLK still reached the peripheral (TUR
// latched from the first clock edge) but the receive shifter
// had no SS-falling-edge to frame against, and RFE never
// cleared.
#define PA_SPI2_FER_MASK 0x003FU // PA0..PA5
// PORTA_MUX packs two bits per pin starting at bit 0 for PA0.
#define PA0_MUX_POS 0U
#define PA1_MUX_POS 2U
#define PA2_MUX_POS 4U
#define PA3_MUX_POS 6U
#define PA4_MUX_POS 8U
#define PA5_MUX_POS 10U
#define SPI2_ALT_FN 1U
#define PA_SPI2_MUX_MASK                                                       \
   ((3U << PA0_MUX_POS) | (3U << PA1_MUX_POS) | (3U << PA2_MUX_POS) |          \
    (3U << PA3_MUX_POS) | (3U << PA4_MUX_POS) | (3U << PA5_MUX_POS))
#define PA_SPI2_MUX_VAL                                                        \
   ((SPI2_ALT_FN << PA0_MUX_POS) | (SPI2_ALT_FN << PA1_MUX_POS) |              \
    (SPI2_ALT_FN << PA2_MUX_POS) | (SPI2_ALT_FN << PA3_MUX_POS) |              \
    (SPI2_ALT_FN << PA4_MUX_POS) | (SPI2_ALT_FN << PA5_MUX_POS))

// enum spi_miom values that matter to the pinmux (kept in sync
// with common/spi.h :: enum spi_miom; a local copy avoids a
// header dependency cycle between pinmux.h and spi.h).
#define PINMUX_MIOM_SINGLE 0U
#define PINMUX_MIOM_DUAL   1U
#define PINMUX_MIOM_QUAD   2U

void pinmux_spi2(int is_master, unsigned miom)
{
   pinmux_spi2_dir(is_master, miom, 0);
}

// Extended: is_tx selects slave-TX direction (data lanes become
// outputs driven by the SPI shifter).  Ignored in master role
// since the master always drives its TX lanes.
void pinmux_spi2_dir(int is_master, unsigned miom, int is_tx)
{
   uint32_t mux = MMR(REG_PORTA_MUX);
   mux &= ~PA_SPI2_MUX_MASK;
   mux |= PA_SPI2_MUX_VAL;
   MMR(REG_PORTA_MUX) = mux;

   // PORT_FER "impact[s] output control only. Regardless of the
   // setting of the function enable bits, both GPIO and
   // peripherals can still sense the pin input." (HRM 18544.)
   //
   // Therefore enable FER bits only for the pins the peripheral
   // actually needs to drive as outputs, and leave pure-input
   // pins with FER=0. Empirically on this chip, forcing FER=1 on
   // a slave's MOSI/SCLK/SS input pins stops the peripheral's
   // internal receiver from ever promoting a shift-register
   // value into SPI_RFIFO -- RFE stays latched for the full
   // transfer. Walking the slave's input pins back to FER=0
   // (GPIO mode, with the peripheral still sensing the pad per
   // the HRM quote above) lets the slave frame every byte.
   //
   // Master-mode outputs (MOSI, CLK, SEL1): FER=1, MUX=1. In
   //   dual/quad master the data lanes (PA0=D1, PA2=D2, PA3=D3)
   //   are also outputs so the full PA0..PA5 mask applies.
   // Master-mode inputs (MISO in single-lane, and slave-SS-as-
   //   master-MODF in): FER can be 0 and the peripheral still
   //   reads the pin. We set the full mask in master role; MODF
   //   is disabled elsewhere so the PA5 FER bit is harmless.
   // Slave single-lane: MISO (PA0) is the one driven output;
   //   everything else is an input. FER = 1 << 0.
   // Slave dual-lane RX: PA0 (D1) and PA1 (D0) are both inputs
   //   from the external master. FER must be 0 on both or the
   //   DSP's pad driver contends with the host.
   // Slave quad-lane RX: PA0..PA3 (D1/D0/D2/D3) are all inputs.
   //   FER = 0 on all four.
   uint32_t fer = MMR(REG_PORTA_FER);
   fer &= ~PA_SPI2_FER_MASK;
   if (is_master) {
      fer |= PA_SPI2_FER_MASK;
   } else if (is_tx) {
      // Slave transmit: data lanes become DSP outputs.
      //   Single: PA0 (MISO) out.
      //   Dual:   PA0 (D1) + PA1 (D0) out.
      //   Quad:   PA0..PA3 (D1/D0/D2/D3) all out.
      // CLK (PA4) and SS (PA5) stay inputs (slave role).
      if (miom == PINMUX_MIOM_SINGLE)
         fer |= (1U << 0U);
      else if (miom == PINMUX_MIOM_DUAL)
         fer |= (1U << 0U) | (1U << 1U);
      else
         fer |= (1U << 0U) | (1U << 1U) | (1U << 2U) | (1U << 3U);
   } else if (miom == PINMUX_MIOM_SINGLE) {
      fer |= (1U << 0U); // PA0 (MISO) output only
   } else {
      // dual/quad slave receive: every data lane is an input,
      // leave FER clear on PA0..PA3. CLK/SS (PA4/PA5) are
      // inputs in every slave variant so they stay FER=0 too.
   }
   MMR(REG_PORTA_FER) = fer;

   // HRM 12-49: PORTA_INEN[n] enables the pad's input buffer.
   // Without INEN=1 the peripheral input connected to that pin
   // reads 0 regardless of the external signal, and the SPI
   // slave receive path never sees SCLK/MOSI/SS edges. FER
   // only governs the OUTPUT direction driver (HRM 12-41: "the
   // function enable bits impact output control only"), so it
   // is orthogonal to INEN -- both must be correct. Master role
   // still needs INEN on MISO + SEL1 (MODF detect), so enable
   // INEN for the whole SPI2 pin group unconditionally.
   MMR(REG_PORTA_INEN_SET) = PA_SPI2_FER_MASK;
}
