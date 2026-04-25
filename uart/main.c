// SPDX-License-Identifier: MIT
// main.c --- UART demo for the EV-21569-SOM
// Copyright (c) 2026 Jakob Kastelic

// Brings up UART0 at 115200 8N1 and prints sequentially
// numbered lines as fast as possible.  Sequential numbering
// lets the receiver detect any dropped bytes.
//
// board_som_init pulls the MCP23017's GPA5 (*UART0_EN) LOW so
// UART0 TX is actually routed to the carrier-board header.
// Without that the MCU puts bytes on the UART pin but the '125
// gate on the SOM leaves them stranded.

#include "board.h"
#include "clocks.h"
#include "timer.h"
#include "uart.h"
#include <stdint.h>
#include <stdio.h>

#define TICK_MS 1000U

int main(void)
{
   static const struct clocks_cfg clk = BOARD_CLOCKS_CFG;
   clocks_init(&clk);
   uart_init(BOARD_BAUD_DIV);
   timer_init();
   board_som_init(0U);
   printf("\r\nuart demo starting\r\n");

   uint32_t n = 0U;
   for (;;) {
      printf("%x\r\n", n);
      n++;
      delay_ms(TICK_MS);
   }
}
