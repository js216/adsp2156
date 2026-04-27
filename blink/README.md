# Blink

This example blinks the three LEDs on the EV-21569-SOM module via the MCP23017
expander on TWI2 at 7-bit address 0x21.

### Hardware Needed

- EV-21569-SOM module
- EV-SOMCRR-EZLITE board
- USB-C cable

### Test Setup

Connect USB-C cable to the QSPI connector `P2` on the EZLITE board.

Boot the DSP and observe the LED 4, 6, 7 blinking pattern. Make sure the red
LED1 is not illuminated (which would mean `SYS_FAULT` is asserted.).

For automated test: connect LED4 to scope C1 and LED1 to scope C2. The board
must be modified to enable automated reset (connect R172 to S3).

### Automated Test

```
dsp:reset
dsp:uart_open
dsp:boot ldr=@main.ldr
delay ms=3000
scope:capture chans="C1,C2"
dsp:uart_close
```

- Check `test_serv` had no errors
- Check fault LED is off
- Check number of LED blinks is correct
- Check LED blink spacing is correct
