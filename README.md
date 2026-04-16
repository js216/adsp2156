# Open Source Firmware Examples for ADSP-2156

Common files:

- **server**: serve code to remote machine for execution there
- **common**: drivers and headers common to all examples
- **board**: board-specific constants, pinmux, and clock configuration

Examples:

- **blink**: blink LEDs on the SOM
- **uart**: print and echo messages on UART0
- **gpio**: GPIO loopback check using hardware jumpers
- **sport**: SPORT0 internal loopback (LFSR check via DAI SRU)
- **sport_dma**: 8-SPORT DMA loopback integrity + throughput benchmark

### Author

Jakob Kastelic (Stanford Research Systems, Inc.)
