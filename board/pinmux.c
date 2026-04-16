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
