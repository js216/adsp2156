# SPORT DMA

This example exercises SPORT4 over an external loopback driven by autobuffer
DMA. Half-A drives ACLK / AFS / AD0 out at one CLKDIV (selected at compile
time via `-DSWEEP_CLKDIV=N`, default 91); the EZLITE P13 jumpers route those
lines back into half-B, which captures them via DMA. Each round fills the TX
buffer with a fresh LFSR pattern, runs SPORT4 long enough for several buffer
wraps, and verifies the RX buffer word-for-word with a rotation offset. The
final UART line reports `PASS bytes=...` (or `FAIL ... mm=...`) for the
build's CLKDIV.

### Hardware Needed

- EV-21569-SOM module
- EV-SOMCRR-EZLITE board
- USB-C cable
- Micro-USB cable

(The manual external sweep additionally needs three jumpers on EZLITE
connector `P13` to loop SPORT4 half-A out to half-B in (ACLK, AFS, AD0).)

### Test Setup

Connect USB-C cable to the QSPI connector `P2` on the EZLITE board. Connect
the Micro-USB cable to the UART connector `P2` on the SOM board. This
automated test uses an internal SRU loopback inside the DSP (built with
`-DUSE_INTERNAL`), so no `P13` jumpers are required.

Boot the DSP. The UART line should print `sport4_ext clkdiv=...` followed by
per-round status and a final `PASS` line. Make sure red LED1 is off (no
`SYS_FAULT`).

For automated test: connect LED1 to scope C2. The board must be modified to
enable automated reset (connect R172 to S3).

### Automated Test

```
dsp:reset
dsp:uart_open
dsp:boot ldr=@main.ldr
delay ms=40000
scope:capture chans="C2"
dsp:uart_close
```

- Check `test_serv` had no errors
- Check fault LED is off
- Check loopback passes

### Manual sweep

The `sweep_runner.py` script in this directory drives an external multi-build
sweep across CLKDIV values, rebuilding the firmware for each candidate and
submitting it via `test_serv/submit.py`. It is a separate harness from the
single-build automated test above and is not invoked by `run_md.py`.
