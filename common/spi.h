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

// SPI port configuration. All fields are 32-bit; smaller field
// widths (enum, uint16_t, bitfields) trigger a cc21k code-gen
// bug on SHARC+ where stack writes use a halfword store that
// the core traps on. See qspi/TODO.md for the original report.
struct spi_cfg {
   uint32_t clkdiv;    // baud = SCLK0 / (clkdiv + 1)
   uint32_t size;      // enum spi_word_size: 8 / 16 / 32-bit
   uint32_t miom;      // enum spi_miom: single / dual / quad
   uint32_t is_master; // 1 = master, 0 = slave
   uint32_t cpol;      // clock polarity
   uint32_t cpha;      // clock phase
   uint32_t lsb_first; // 0 = MSB first, 1 = LSB first
};

// Initialise the SPI module with the given configuration. The
// module is left enabled but TX/RX are not started; call
// spi_tx_enable / spi_rx_enable to begin transfers.
void spi_init(enum spi_id id, const struct spi_cfg *cfg);

// Enable TX and RX independently. In master mode enabling
// TX or RX with TTI/RTI starts clocking immediately.
void spi_tx_enable(enum spi_id id);
void spi_rx_enable(enum spi_id id);

// Bring the SPI module to a quiescent state and clear SPI_CTL.
// HRM 15-12 requires the SPI to be quiescent before a MIOM change
// (single / dual / quad); without that guarantee the new lane
// config can be applied mid-word and the opening bytes of the
// first transfer in the new mode go missing.  Polls SPI_STAT.SPIF
// (with a bounded timeout so a wedged slave can't hang the shell)
// and then drops CTL.EN.  Also W1Cs the sticky status bits so the
// next spi_init starts from a clean slate.
//   id: which SPI controller.
void spi_disable(enum spi_id id);

// Enable the RX DMA request line. The SPI peripheral will
// request a DMA transfer whenever its RX FIFO is non-empty,
// allowing the SPI0/1/2_RX DMA channel (22/24/26) to drain
// the FIFO without core intervention. Must be called before
// spi_rx_enable if DMA is to be used.
void spi_rx_dma_enable(enum spi_id id);

#endif // SPI_H
