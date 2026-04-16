// SPDX-License-Identifier: MIT
// uart.c --- Polled UART0 driver
// Copyright (c) 2026 Jakob Kastelic

#include "uart.h"
#include "pinmux.h"
#include "regs.h"
#include <stdint.h>
#include <stdio.h>

void uart_init(const uint32_t baud_div)
{
   pinmux_uart0();

   // Disable the UART before reprogramming.
   MMR(REG_UART0_CTL) = 0U;

   // EDBO = 1 selects divide-by-one mode; the divisor is SCLK / baud.
   // The caller has already done that math.
   MMR(REG_UART0_CLK) = BIT_UART_CLK_EDBO | (baud_div & MASK_UART_CLK_DIV);

   // 8 data bits, 1 stop bit, no parity, UART mode, then enable.
   MMR(REG_UART0_CTL) = BIT_UART_CTL_WLS_8 | BIT_UART_CTL_STB_1 |
                        BIT_UART_CTL_PEN_OFF | BIT_UART_CTL_MOD_UART |
                        BIT_UART_CTL_EN;
}

void uart_putc(const char c)
{
   while ((MMR(REG_UART0_STAT) & BIT_UART_STAT_THRE) == 0U)
      ;
   MMR(REG_UART0_THR) = (uint32_t)(uint8_t)c;
}

int uart_try_getc(void)
{
   if ((MMR(REG_UART0_STAT) & BIT_UART_STAT_DR) == 0U) {
      return -1;
   }
   return (int)(uint8_t)MMR(REG_UART0_RBR);
}

// Provide the per-byte hook printf expects.
int putchar(const int c)
{
   uart_putc((char)c);
   return c;
}
