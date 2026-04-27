// SPDX-License-Identifier: MIT
// main.c --- UART demo for the EV-21569-SOM
// Copyright (c) 2026 Jakob Kastelic

// Brings up UART0 at 115200 8N1 and emits a banner followed by
// sequentially numbered hex lines once per TICK_MS so the
// receiver can detect dropped bytes.
//
// board_som_init pulls MCP23017 GPA5 (*UART0_EN) LOW so UART0 TX
// reaches the carrier-board header.

#include "board.h"
#include "clocks.h"
#include "timer.h"
#include "uart.h"
#include <stdint.h>

#define TICK_MS         300U
#define HEX_NIBBLE_BITS 4U
#define HEX_NIBBLE_MASK 0xFU
#define HEX_DIGITS      8U
#define DEC_RADIX       10U

static void emit_str(const char *s)
{
   while (*s) {
      uart_putc(*s++);
   }
}

static void emit_hex32(uint32_t v)
{
   for (unsigned i = 0U; i < HEX_DIGITS; i++) {
      unsigned shift = (HEX_DIGITS - 1U - i) * HEX_NIBBLE_BITS;
      unsigned nib   = (v >> shift) & HEX_NIBBLE_MASK;
      uart_putc(
          (char)((nib < DEC_RADIX) ? ('0' + nib) : ('a' + (nib - DEC_RADIX))));
   }
}

int main(void)
{
   static const struct clocks_cfg clk = BOARD_CLOCKS_CFG;
   clocks_init(&clk);
   uart_init(BOARD_BAUD_DIV);
   timer_init();
   board_som_init(0U);
   emit_str("\r\nuart demo starting\r\n");

   uint32_t n = 0U;
   for (;;) {
      emit_hex32(n);
      emit_str("\r\n");
      n++;
      delay_ms(TICK_MS);
   }
}
