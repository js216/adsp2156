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

// SPI2 QSPI pins: PA0..PA5, all mux function "b" (value 1).
// Pin assignments (HRM Table 12-61, PORTA multiplexer):
//   PA0 = SPI2_MISO  (D1)
//   PA1 = SPI2_MOSI  (D0)
//   PA2 = SPI2_D2
//   PA3 = SPI2_D3
//   PA4 = SPI2_CLK
//   PA5 = SPI2_SEL1
#define PA_SPI2_FER_MASK 0x003FU // PA0..PA5
#define PA_SPI2_MUX_MASK                                                       \
   ((3U << 0U) | (3U << 2U) | (3U << 4U) | (3U << 6U) | (3U << 8U) |           \
    (3U << 10U))
#define PA_SPI2_MUX_VAL                                                        \
   ((1U << 0U) | (1U << 2U) | (1U << 4U) | (1U << 6U) | (1U << 8U) |           \
    (1U << 10U))

void pinmux_spi2(void)
{
   uint32_t mux = MMR(REG_PORTA_MUX);
   mux &= ~PA_SPI2_MUX_MASK;
   mux |= PA_SPI2_MUX_VAL;
   MMR(REG_PORTA_MUX) = mux;
   MMR(REG_PORTA_FER) |= PA_SPI2_FER_MASK;
}
