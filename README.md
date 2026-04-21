# Open Source Firmware Examples for ADSP-2156

Common files:

- **common**: drivers, stdlib headers, board-specific constants,
  pinmux, and clock configuration shared by every example

Examples:

- **blink**: blink LEDs on the SOM
- **uart**: print and echo messages on UART0
- **gpio**: P13/P14 header connectivity tester (drive/sense sweep)
- **sport**: SPORT0 internal loopback (LFSR check via DAI SRU)
- **sport_dma**: 8-SPORT DMA loopback integrity + throughput benchmark
- **fir**: hardware FIR accelerator low-pass filter demo
- **iir**: hardware IIR accelerator biquad low-pass filter demo
- **qspi_slave**: polled SPI2 slave echo demo for FTDI USB master

### Author

Jakob Kastelic (Stanford Research Systems, Inc.)
