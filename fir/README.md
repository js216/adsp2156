# FIR

This example exercises the hardware FIR accelerator on the ADSP-21569. It
generates a 200 Hz + 4 kHz two-tone signal (48 kHz sample rate), filters it
through a 17-tap low-pass FIR (cutoff ~1 kHz, Hamming window), and prints
the IEEE-754 hex bits of the input peak, output peak, and a few steady-state
samples over UART. A final `PASS lpf_attenuation` line confirms the
high-frequency tone was attenuated.

### Hardware Needed

- EV-21569-SOM module
- EV-SOMCRR-EZLITE board
- Micro-USB and USB-C cables

### Test Setup

Connect USB-C cable to the QSPI connector `P2` on the EZLITE board. Connect
the Micro-USB cable to the UART connector `P2` on the SOM board.

Boot the DSP. The UART line should print `fir demo starting`, several lines
of hex values, then `PASS lpf_attenuation` and `fir demo done`. Make sure
red LED1 is off (no `SYS_FAULT`).

For automated test: connect LED1 to scope C2. The board must be modified to
enable automated reset (connect R172 to S3).

### Automated Test

```
dsp:reset
dsp:uart_open
dsp:boot ldr=@main.ldr
delay ms=5000
scope:capture chans="C2"
dsp:uart_close
```

- Check `test_serv` had no errors
- Check fault LED is off
- Check FIR output present
- Check FIR attenuation passed
