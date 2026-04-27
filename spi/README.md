# SPI

This example exercises full-duplex SPI0 <-> SPI1 throughput on the
EV-21569-SOM module.  Both ports run ping-pong DMA: each side owns a
TX ring (refilled with PRBS) and an RX ring (CRC-32'd and compared
against the same PRBS stream).  A UART0 shell drives the run; test
A makes SPI0 the master and SPI1 the slave, test B swaps them, and
each test prints a `PASS` line on success or a `WARN`/`ERR` marker
on failure.

### Hardware Needed

- EV-21569-SOM module
- EV-SOMCRR-EZLITE board
- Micro-USB and USB-C cables
- Four jumpers on connector P14

### Test Setup

Connect the USB-C cable to the QSPI connector `P2` on the EZLITE board,
and the Micro-USB cable to the UART connector `P2` on the SOM board.

Install four jumpers on P14 to loop SPI0 to SPI1:

    P14.02 (SPI0_CLK)  <--> P14.12 (SPI1_CLK)
    P14.04 (SPI0_MISO) <--> P14.14 (SPI1_MISO)
    P14.06 (SPI0_MOSI) <--> P14.16 (SPI1_MOSI)
    P14.08 (SPI0_SSB)  <--> P14.18 (SPI1_SEL1*)

For automated test: connect LED1 to scope C2.  The board must be
modified to enable automated reset (connect R172 to S3).

### Automated Test

```
dsp:reset
dsp:uart_open
dsp:boot ldr=@main.ldr
delay ms=1000
dsp:uart_write data="n 32k\r"
delay ms=300
dsp:uart_write data="a\r"
delay ms=10000
scope:capture chans="C2"
dsp:uart_close
```

- Check `test_serv` had no errors
- Check fault LED is off
- Check SPI shell banner present
- Check SPI test was invoked
