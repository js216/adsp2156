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
   uint16_t clkdiv;         // baud = SCLK0 / (clkdiv + 1)
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

// Enable / disable TX and RX independently. In master mode
// enabling TX or RX with TTI/RTI starts clocking immediately.
void spi_tx_enable(enum spi_id id);
void spi_rx_enable(enum spi_id id);
void spi_tx_disable(enum spi_id id);
void spi_rx_disable(enum spi_id id);

// Assert / deassert slave-select 1 (software control). Only
// meaningful in master mode with ASSEL=0.
void spi_select(enum spi_id id);
void spi_deselect(enum spi_id id);

// Polled single-word write. Spins until the TX FIFO has room,
// then pushes one word. Does not wait for the transfer to
// finish.
void spi_write(enum spi_id id, uint32_t word);

// Sentinel returned by spi_read() on poll timeout.
#define SPI_READ_TIMEOUT 0xDEAD0001U

// Polled single-word read. Spins until the RX FIFO is
// non-empty, then pops and returns one word. Returns
// SPI_READ_TIMEOUT if the poll limit is reached.
uint32_t spi_read(enum spi_id id);

// Polled full-duplex transfer of `n` words. For each word,
// writes from `tx` (or zero if tx is NULL) and reads into `rx`
// (or discards if rx is NULL). Returns 0 on success, -1 on
// error (overrun / underrun / timeout).
int spi_xfer(enum spi_id id, const uint32_t *tx, uint32_t *rx, uint32_t n);

// Disable the SPI module entirely.
void spi_disable(enum spi_id id);

#endif // SPI_H
