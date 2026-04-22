// SPDX-License-Identifier: MIT
// pinmux.h --- Pin-multiplexing helpers for the EV-21569-SOM
// Copyright (c) 2026 Jakob Kastelic

// Centralises all PORTA pin mux operations so that drivers do
// not independently read-modify-write the shared PORTA_FER /
// PORTA_MUX registers and risk clobbering each other's setup.
// Each function enables the requested peripheral on the pins
// it owns and leaves every other pin's mux state untouched.

#ifndef PINMUX_H
#define PINMUX_H

#include <stdint.h>

// Configure PA6 (TX) and PA7 (RX) as UART0 alternate-function
// "b" (mux value 1). Also sets PORTA_FER to hand control of
// those pins to the UART peripheral.
void pinmux_uart0(void);

// Configure PA14 (SCL) and PA15 (SDA) as TWI2 alternate-
// function "a" (mux value 0). Also sets PORTA_FER.
void pinmux_twi2(void);

// Configure PA0..PA5 for SPI2. PA0..PA4 use alternate function
// "b" (mux value 1): PA0 = MISO, PA1 = MOSI, PA2 = D2,
// PA3 = D3, PA4 = CLK. PA5 is role-dependent: master role uses
// function "b" (mux 1, SPI2_SEL1 output) and slave role uses
// function "d" (mux 3, SPI2_SS input). Also sets PORTA_FER to
// hand the pins over to the peripheral.
void pinmux_spi2(int is_master);

#endif // PINMUX_H
