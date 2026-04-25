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
//
// ``miom`` selects the lane width (0 = single, 1 = dual,
// 2 = quad). It governs PORTA_FER on the slave's data pins:
// in single-lane slave role PA0 (MISO) is a driven output and
// keeps FER=1, but in dual/quad slave receive PA0 (D1), PA1
// (D0), and -- for quad only -- PA2/PA3 (D2/D3) are inputs
// from the external master, so FER must stay clear on those
// pins or the DSP's pad driver will fight the host. Master
// role always needs FER=1 on all of PA0..PA5 so the peripheral
// drives CLK/SEL1/MOSI (and D1..D3 when in dual/quad master).
void pinmux_spi2(int is_master, unsigned miom);

// Extended form.  ``is_tx`` selects the slave direction: 0 (slave
// RX) leaves data lanes as inputs in dual/quad; non-zero (slave
// TX) configures them as outputs driven by the SPI shifter.
// Ignored in master role since master always drives its own lanes.
void pinmux_spi2_dir(int is_master, unsigned miom, int is_tx);

// Configure PA6..PA9 for SPI0 (single-lane only): PA6 = CLK,
// PA7 = MISO, PA8 = MOSI, PA9 = SEL1b / SSb.  Alt function "a"
// (mux value 0).  ``is_master`` picks which pins the DSP drives:
// master asserts CLK / MOSI / SEL1 on PA6/PA8/PA9, senses MISO
// on PA7; slave senses CLK/MOSI/SS on PA6/PA8/PA9 and sources
// MISO on PA7.  FER is set on the pin(s) the DSP drives.
void pinmux_spi0(int is_master);

// Configure PA10..PA13 for SPI1 (single-lane only): PA10 = CLK,
// PA11 = MISO, PA12 = MOSI, PA13 = SEL1b / SSb.  Alt function "b"
// (mux value 1).  Same role rules as pinmux_spi0.
void pinmux_spi1(int is_master);

#endif // PINMUX_H
