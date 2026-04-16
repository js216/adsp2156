// SPDX-License-Identifier: MIT
// main.c --- UART demo for the EV-21569-SOM
// Copyright (c) 2026 Jakob Kastelic

// Brings up UART0 at 115200 8N1 via the standard uart_init /
// printf path and emits a couple of lines per second forever.
// Also drains the RX FIFO and echoes incoming bytes back, which
// is enough to verify TX, RX, and the printf format machinery
// from a host terminal.

#include "board.h"
#include "clocks.h"
#include "timer.h"
#include "uart.h"
#include <stdint.h>
#include <stdio.h>

#define TICK_MS 800U

static void rx_drain_and_echo(void)
{
   int c = -1;
   while ((c = uart_try_getc()) >= 0) {
      uart_putc((char)c);
   }
}

int main(void)
{
   static const struct clocks_cfg clk = BOARD_CLOCKS_CFG;
   clocks_init(&clk);
   uart_init(BOARD_BAUD_DIV);
   timer_init();
   printf("\r\nuart demo starting\r\n");

   uint32_t tick = 0U;
   for (;;) {
      printf("tick %x ping\r\n", tick);
      tick++;
      delay_ms(TICK_MS);
      rx_drain_and_echo();

      printf("tick %x pong\r\n", tick);
      tick++;
      delay_ms(TICK_MS);
      rx_drain_and_echo();
   }
}
