# GPIO

This example scans P13 and P14 header pins on the EV-SOMCRR-EZLITE board for
shorts and connectivity anomalies. It drives each testable pin high then low,
samples every other pin, and prints a connectivity report over UART0. A new
scan is triggered whenever the host sends `t` on UART0. The shell also accepts
a few additional one-letter commands (see below).

### Hardware Needed

- EV-21569-SOM module
- EV-SOMCRR-EZLITE board
- Micro-USB and USB-C cables

### Test Setup

Connect USB-C cable to the QSPI connector `P2` on the EZLITE board. Connect the
Micro-USB cable to the UART connector `P2` on the SOM board.

Boot the DSP. The UART line should print a `=== scan 0 ===` banner followed by
the connectivity report. Make sure red LED1 is off (no `SYS_FAULT`).

For automated test: connect LED1 to scope C2. The EZLITE board must be modified
to enable automated reset (connect R172 to S3).

### Manual Test

After the boot scan completes, the demo accepts these single-character commands
on UART0:

| key       | action                                                          |
|-----------|-----------------------------------------------------------------|
| `t`       | run another connectivity scan                                   |
| `?` / `h` | print the command list and the pin index table                  |
| `b<idx>`  | blink one pin for a few seconds (`<idx>` is the next character) |

Pin indices are encoded as one ASCII character: `0`..`9` for pins 0..9 and
`a`..`n` for pins 10..23. Send `?` first to see the full pin index table the
firmware prints; the table maps each index character to its `Pxx.yy` header
position and the silicon signal name. To blink P13.02 (pin 0) type `b0`; to
blink P14.20 (pin 9) type `b9`; to blink P14.27 (pin 23) type `bn`. Each blink
toggles the pin 10 times at 2 Hz, then parks it back as input.

### Automated Test

```
dsp:reset
dsp:uart_open
dsp:boot ldr=@main.ldr
delay ms=500
dsp:uart_write data="?\r"
delay ms=250
dsp:uart_write data="t\r"
delay ms=250
scope:capture chans="C2"
dsp:uart_close
```

- Check `test_serv` had no errors
- Check fault LED is off
- Check UART scan output present
- Check scan completed
- Check help output present
- Check trigger command runs second scan
