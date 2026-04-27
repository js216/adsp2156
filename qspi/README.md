# QSPI

This example brings up SPI2 as a slave receiver on PA0..PA5 and exposes an
interactive UART0 shell that drains received data as raw hex. The shell takes
single-letter commands (`m1`/`m2`/`m4` for lane width, `b <bytes>` for DMA ring
size, `h [n]` to hex-dump received words, `i` to print the running CRC, etc.).

The automated test exercises the canonical bring-up path: configure quad mode,
size the DMA ring, arm a 64-byte hex-dump, and have the host master clock 64
bytes of PRBS over SPI. The DSP prints sixteen `w<idx>=0x<hex>` lines.

### Hardware Needed

- EV-21569-SOM module
- EV-SOMCRR-EZLITE board
- Micro-USB and USB-C cables
- FT4222 USB-to-SPI adapter wired as SPI master to the EZLITE QSPI header
  (D0..D3 to PA1/PA0/PA2/PA3, CLK to PA4, CS to PA5)

### Test Setup

Connect USB-C cable to the QSPI connector `P2` on the EZLITE board. Connect the
Micro-USB cable to the UART connector `P2` on the SOM board. Wire the FT4222
master to the EZLITE QSPI lane group (PA0..PA5).

Boot the DSP. The UART line should print `qspi shell starting` followed by
`ready. type \`help\` for commands.` and a `> ` prompt. Make sure red LED1 is
off (no `SYS_FAULT`).

For automated test: connect LED1 to scope C2. The board must be modified to
enable automated reset (connect R172 to S3).

### Automated Test

```
dsp:reset
dsp:uart_open
dsp:boot ldr=@main.ldr
delay ms=500
dsp:uart_write data="m4\r"
delay ms=50
dsp:uart_write data="b 128\r"
delay ms=50
dsp:uart_write data="?\r"
delay ms=200
dsp:uart_write data="h 64\r"
delay ms=200
mark tag=send_start
dsp:qspi_write_prbs seed=0xC0FFEE n=64 clk_div=8 mode=4 chunk_size=64
mark tag=send_end
dsp:uart_expect sentinel="w0000000f=" timeout_ms=5000
scope:capture chans="C2"
dsp:uart_close
```

- Check `test_serv` had no errors
- Check fault LED is off
- Check expected UART output present
