// SPDX-License-Identifier: MIT
// main.c --- SPI2 slave demo for the EV-21569-SOM (FTDI USB master)
// Copyright (c) 2026 Jakob Kastelic

// Configures SPI2 as a polled slave (mode 0, 8-bit, single I/O)
// and waits for an external FTDI USB-SPI adapter acting as
// master. Each byte received from the master is printed over
// UART in hex; the slave echoes the previous byte back to the
// master on the next transfer (the very first reply is 0x00).
//
// Pin mapping (PA0..PA5 alternate function "b"):
//   PA0 = SPI2_MISO   (slave output)
//   PA1 = SPI2_MOSI   (slave input)
//   PA4 = SPI2_CLK    (clock from master)
//   PA5 = SPI2_SEL1   (chip-select from master, active low)
//
// FTDI MPSSE default is SPI mode 0 (CPOL=0, CPHA=0), MSB first.

#include "board.h"
#include "clocks.h"
#include "pinmux.h"
#include "regs.h"
#include "spi.h"
#include "timer.h"
#include "uart.h"
#include <stdint.h>
#include <stdio.h>

#define SPI_PORT SPI_ID_2

// Print the SPI_STAT register with decoded error flags.
static void print_stat(enum spi_id id)
{
   uint32_t base = REG_SPI0_BASE + (uint32_t)id * 0x1000U;
   uint32_t s    = MMR(base + OFF_SPI_STAT);
   printf("  stat %08x", s);
   if (s & BIT_SPI_STAT_ROR)
      printf(" ROR");
   if (s & BIT_SPI_STAT_TUR)
      printf(" TUR");
   if (s & BIT_SPI_STAT_TC)
      printf(" TC");
   printf("\r\n");
}

int main(void)
{
   static const struct clocks_cfg clk = BOARD_CLOCKS_CFG;
   clocks_init(&clk);
   uart_init(BOARD_BAUD_DIV);
   timer_init();
   delay_ms(500);

   printf("\r\nqspi_slave demo starting\r\n");

   pinmux_spi2();

   struct spi_cfg cfg = {
       .clkdiv    = 0,
       .size      = SPI_WORD_8,
       .miom      = SPI_MIO_SINGLE,
       .is_master = 0,
       .cpol      = 0,
       .cpha      = 0,
       .lsb_first = 0,
   };
   spi_init(SPI_PORT, &cfg);

   // Pre-load the very first response byte (master will
   // receive this while sending its first byte to us).
   spi_write(SPI_PORT, 0x00U);

   spi_tx_enable(SPI_PORT);
   spi_rx_enable(SPI_PORT);

   printf("slave ready, waiting for master\r\n");
   print_stat(SPI_PORT);

   uint32_t count = 0;
   for (;;) {
      uint32_t rx = spi_read(SPI_PORT);
      if (rx == 0xDEAD0001U) {
         // Poll timeout -- master has not sent anything yet.
         // Re-arm and try again after a short delay.
         delay_ms(500);
         printf(".");
         continue;
      }

      printf("rx[%x] %02x\r\n", count, rx & 0xFFU);
      count++;

      // Load the echo byte for the next transfer.
      spi_write(SPI_PORT, rx);

      // Print status periodically so errors are visible.
      if ((count & 0xFU) == 0)
         print_stat(SPI_PORT);
   }
}
