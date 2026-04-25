# Open Source Firmware Examples for ADSP-2156

Examples:

- **blink**: blink LEDs on the SOM
- **uart**: print and echo messages on UART0
- **gpio**: P13/P14 header connectivity tester (drive/sense sweep)
- **sport**: SPORT0 internal loopback (LFSR check via DAI SRU)
- **sport_dma**: 8-SPORT DMA loopback integrity + throughput benchmark
- **fir**: hardware FIR accelerator low-pass filter demo
- **iir**: hardware IIR accelerator biquad low-pass filter demo
- **qspi**: SPI2 PRBS throughput and integrity harness

Drivers, board-specific constants, pinmux, and clock configuration shared by
every example is located under **common/**.

To avoid including ADI-proprietary libraries and headers, we make use of the
[`libsel`](https://github.com/js216/selache/tree/main/libsel) from the
[Selache](https://github.com/js216/selache) project.

The readme file in each example explains the hardware and procedure needed to
test manually, as well as a description of the automated test procedure using
the [`test_serv`](https://github.com/js216/test_serv) framework.

### Author

Jakob Kastelic (Stanford Research Systems, Inc.)
