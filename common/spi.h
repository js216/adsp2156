// SPDX-License-Identifier: MIT
// spi.h --- Polled SPI driver for the ADSP-2156x
// Copyright (c) 2026 Jakob Kastelic

#ifndef SPI_H
#define SPI_H

#include <stdint.h>

// SPI module identifiers. SPI2 is the only one that supports
// quad I/O mode on the ADSP-2156x.
enum spi_id {
   SPI_ID_0 = 0,
   SPI_ID_1 = 1,
   SPI_ID_2 = 2,
};

// Word transfer size.
enum spi_word_size {
   SPI_WORD_8  = 0,
   SPI_WORD_16 = 1,
   SPI_WORD_32 = 2,
};

// Multiple-I/O mode.
enum spi_miom {
   SPI_MIO_SINGLE = 0, // standard single-lane SPI
   SPI_MIO_DUAL   = 1, // dual I/O (SPI1/SPI2)
   SPI_MIO_QUAD   = 2, // quad I/O (SPI2 only)
};

// SPI port configuration.
struct spi_cfg {
   uint32_t clkdiv;         // baud = SCLK0 / (clkdiv + 1)
   enum spi_word_size size; // 8, 16, or 32-bit words
   enum spi_miom miom;      // single / dual / quad
   unsigned is_master : 1;  // 1 = master, 0 = slave
   unsigned cpol : 1;       // clock polarity
   unsigned cpha : 1;       // clock phase
   unsigned lsb_first : 1;  // 0 = MSB first, 1 = LSB first
};

// Initialise the SPI module with the given configuration. The
// module is left enabled but TX/RX are not started; call
// spi_tx_enable / spi_rx_enable to begin transfers.
void spi_init(enum spi_id id, const struct spi_cfg *cfg);

// Enable TX and RX independently. In master mode enabling
// TX or RX with TTI/RTI starts clocking immediately.
void spi_tx_enable(enum spi_id id);
void spi_rx_enable(enum spi_id id);

// Enable the RX DMA request line. The SPI peripheral will
// request a DMA transfer whenever its RX FIFO is non-empty,
// allowing the SPI0/1/2_RX DMA channel (22/24/26) to drain
// the FIFO without core intervention. Must be called before
// spi_rx_enable if DMA is to be used.
void spi_rx_dma_enable(enum spi_id id);

#endif // SPI_H
