# IIR

This example exercises the hardware IIR accelerator on the EV-21569-SOM. It
generates a 256-sample two-tone test signal (200 Hz + 4 kHz at 48 kHz),
runs it through a 2nd-order Butterworth low-pass biquad (cutoff ~1 kHz)
via the IIR accelerator, and prints input peak, status word, steady-state
peak, and a few output sample IEEE-754 hex words over UART. A line of the
form `PASS iir_attenuation` is printed when the high-frequency tone is
attenuated as expected.

### Hardware Needed

- EV-21569-SOM module
- EV-SOMCRR-EZLITE board
- Micro-USB and USB-C cables

### Test Setup

Connect USB-C cable to the QSPI connector `P2` on the EZLITE board. Connect
the Micro-USB cable to the UART connector `P2` on the SOM board.

Boot the DSP. The UART line should print `iir demo starting`, several lines
with hex IEEE-754 sample bits, `PASS iir_attenuation`, and finally
`iir demo done`. Make sure red LED1 is off (no `SYS_FAULT`).

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
- Check IIR banner present
- Check IIR output samples present
- Check IIR attenuation passed
