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

// UART_STAT error bits.  Writing 1 clears the latched flag; on the
// ADSP-2156x, leaving OE (or any of the other line-error flags)
// latched blocks subsequent RX -- the UART stops asserting DR until
// the error is acknowledged by software.  Clearing them on every poll
// keeps RX alive after a transient overrun instead of silently
// dropping every byte forever.
#define BIT_UART_STAT_OE (1U << 1U) // overrun
#define BIT_UART_STAT_PE (1U << 2U) // parity
#define BIT_UART_STAT_FE (1U << 3U) // framing
#define BIT_UART_STAT_BI (1U << 4U) // break interrupt
#define UART_STAT_RX_ERRS                                                      \
   (BIT_UART_STAT_OE | BIT_UART_STAT_PE | BIT_UART_STAT_FE | BIT_UART_STAT_BI)

int uart_try_getc(void)
{
   uint32_t stat = MMR(REG_UART0_STAT);
   if (stat & BIT_UART_STAT_DR) {
      return (int)(uint8_t)MMR(REG_UART0_RBR);
   }
   // Only touch the STAT register when there is nothing to read;
   // W1C on an active RX path risks clearing DR as a side effect on
   // some pipelining and starving the next byte.
   if (stat & UART_STAT_RX_ERRS) {
      MMR(REG_UART0_STAT) = stat & UART_STAT_RX_ERRS;
   }
   return -1;
}

// Provide the per-byte hook printf expects.
int putchar(const int c)
{
   uart_putc((char)c);
   return c;
}
