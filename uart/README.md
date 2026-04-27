# UART

This example brings up UART0 at 115200 8N1 and prints sequential hex numbers
once per second so a host can verify the SOM-to-carrier UART path and detect
dropped bytes.

### Hardware Needed

- EV-21569-SOM module
- EV-SOMCRR-EZLITE board
- Micro-USB and USB-C cables

### Test Setup

Connect USB-C cable to the QSPI connector `P2` on the EZLITE board. Connect the
Micro-USB cable to the UART connector `P2` on the SOM board.

Boot the DSP. The UART line should print `uart demo starting` followed by an
incrementing hex counter, one line per second. Make sure red LED1 is off (no
`SYS_FAULT`).

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
- Check UART banner present
- Check enough ticks have been printed
- Check UART tick numbers are sequential
