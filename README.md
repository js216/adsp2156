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

Drivers, board-specific constants, pinmux, clock configuration, and reusable
test code, shared by every example, are located under **common/**.

To avoid including ADI-proprietary libraries and headers, we make use of the
[`libsel`](https://github.com/js216/selache/tree/main/libsel) from the
[Selache](https://github.com/js216/selache) project.

### How to Test

The readme file in each example explains the hardware and procedure needed to
test manually, as well as a description of the automated test procedure using
the [`test_serv`](https://github.com/js216/test_serv) framework.

Each example defines a `verify.py` script which makes use of the bullet points
following each automated test description to check that the returned test
artifacts pass the test criteria.

With the test server running, each example can be verified in one step. For
example:

    cd blink
    python3 $TEST_SERV/run_md.py

### Author

Jakob Kastelic (Stanford Research Systems, Inc.)
