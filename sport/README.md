# SPORT

This example exercises the ADSP-21569 SPORT driver with two back-to-back
loopback datapath tests: first SPORT0 internal loopback (no wires, routed
through the DAI0 SRU crossbar) and then SPORT4 external loopback over three
jumpers on the EZLITE carrier's `P13` header. Each phase pushes a 32-bit
LFSR sequence through the TX half and re-derives the expected stream on the
RX half. Per-phase PASS/FAIL is reported over UART.

### Hardware Needed

- EV-21569-SOM module
- EV-SOMCRR-EZLITE board
- Micro-USB and USB-C cables
- Three jumpers on `P13` for the SPORT4 external loopback

### Test Setup

Connect USB-C cable to the QSPI connector `P2` on the EZLITE board. Connect
the Micro-USB cable to the UART connector `P2` on the SOM board.

Install the three SPORT4 loopback jumpers on `P13`:

    P13 pin 2  <-> P13 pin 10   (serial data)
    P13 pin 6  <-> P13 pin 14   (serial clock)
    P13 pin 8  <-> P13 pin 16   (frame sync)

Boot the DSP. The UART line should print one line per phase ending in `PASS`
or `FAIL`. Make sure red LED1 is off (no `SYS_FAULT`).

For automated test: connect `SYS_FAULT` / LED1 to scope channel C2. The
board must be modified to enable automated reset (connect R172 to S3).

### Automated Test

```
dsp:reset
dsp:uart_open
dsp:boot ldr=@main.ldr
delay ms=30000
scope:capture chans="C2"
dsp:uart_close
```

- Check `test_serv` had no errors
- Check fault LED is off
- Check loopback sweep had no failures or timeouts
