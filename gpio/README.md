# GPIO

This example scans P13 and P14 header pins on the EV-SOMCRR-EZLITE board for
shorts and connectivity anomalies. It drives each testable pin high then low,
samples every other pin, and prints a connectivity report over UART0. A new
scan is triggered whenever the host sends `t` on UART0.

### Hardware Needed

- EV-21569-SOM module
- EV-SOMCRR-EZLITE board
- Micro-USB and USB-C cables

### Test Setup

Connect USB-C cable to the QSPI connector `P2` on the EZLITE board. Connect the
Micro-USB cable to the UART connector `P2` on the SOM board.

Boot the DSP. The UART line should print a `=== scan 0 ===` banner followed by
the connectivity report. Make sure red LED1 is off (no `SYS_FAULT`).

For automated test: connect LED1 to scope C2. The board must be modified to
enable automated reset (connect R172 to S3).

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
- Check UART scan output present
- Check scan completed
