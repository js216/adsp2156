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

// SPI2 pins: PA0..PA5, alternate function "b" (mux value 1) on
// PA0..PA4. In master role PA5 is SPI2_SEL1 (output, also alt
// "b"); in slave role PA5 is SPI2_SS (input, alternate function
// "d", mux value 3). Empirically verified by SELFTEST: PA0..PA4
// at mux=1 with the SPI2 peripheral in master mode drives CLK
// and returns full-duplex RX on MISO.
#define PA_SPI2_FER_MASK 0x003FU // PA0..PA5
// PORTA_MUX packs two bits per pin starting at bit 0 for PA0.
#define PA0_MUX_POS         0U
#define PA1_MUX_POS         2U
#define PA2_MUX_POS         4U
#define PA3_MUX_POS         6U
#define PA4_MUX_POS         8U
#define PA5_MUX_POS         10U
#define SPI2_ALT_FN         1U // alternate function "b" on PA0..PA4
#define PA5_SLAVE_SS_ALT_FN 3U // alternate function "d" on PA5 = SPI2_SS input
#define PA5_MASTER_SEL1_ALT_FN                                                 \
   1U // alternate function "b" on PA5 = SPI2_SEL1 output
#define PA_SPI2_MUX_MASK                                                       \
   ((3U << PA0_MUX_POS) | (3U << PA1_MUX_POS) | (3U << PA2_MUX_POS) |          \
    (3U << PA3_MUX_POS) | (3U << PA4_MUX_POS) | (3U << PA5_MUX_POS))

void pinmux_spi2(int is_master)
{
   uint32_t pa0_4 = (SPI2_ALT_FN << PA0_MUX_POS) |
                    (SPI2_ALT_FN << PA1_MUX_POS) |
                    (SPI2_ALT_FN << PA2_MUX_POS) |
                    (SPI2_ALT_FN << PA3_MUX_POS) | (SPI2_ALT_FN << PA4_MUX_POS);
   uint32_t pa5_field =
       (is_master ? PA5_MASTER_SEL1_ALT_FN : PA5_SLAVE_SS_ALT_FN)
       << PA5_MUX_POS;
   uint32_t mux = MMR(REG_PORTA_MUX);
   mux &= ~PA_SPI2_MUX_MASK;
   mux |= pa0_4 | pa5_field;
   MMR(REG_PORTA_MUX) = mux;
   MMR(REG_PORTA_FER) |= PA_SPI2_FER_MASK;
}
