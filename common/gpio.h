// SPDX-License-Identifier: MIT
// gpio.h --- General-purpose GPIO API covering PORTA/B/C + DAI0/DAI1
// Copyright (c) 2026 Jakob Kastelic

// The ADSP-21569 exposes GPIO-capable pins through the
// PORTA/B/C blocks (16 pads each) and the two DAI units' pin
// buffers (14 pads each); this driver hides both behind one
// enum and a single API. Pins that do not physically exist on
// the part (PB13..PB18 on each DAI) are simply not in the
// enum, and passing an out-of-range value trips an assertion.
// Calls against pads already multiplexed to an active
// peripheral (UART, TWI, SPORT, ...) contend with that
// peripheral and are the caller's responsibility.

#ifndef GPIO_H
#define GPIO_H

#include <stdbool.h>
#include <stdint.h>

enum gpio_pin {
   // PORTA pins PA00..PA15
   GPIO_PA00 = 0,
   GPIO_PA01,
   GPIO_PA02,
   GPIO_PA03,
   GPIO_PA04,
   GPIO_PA05,
   GPIO_PA06,
   GPIO_PA07,
   GPIO_PA08,
   GPIO_PA09,
   GPIO_PA10,
   GPIO_PA11,
   GPIO_PA12,
   GPIO_PA13,
   GPIO_PA14,
   GPIO_PA15,

   // PORTB pins PB00..PB15
   GPIO_PB00 = 16,
   GPIO_PB01,
   GPIO_PB02,
   GPIO_PB03,
   GPIO_PB04,
   GPIO_PB05,
   GPIO_PB06,
   GPIO_PB07,
   GPIO_PB08,
   GPIO_PB09,
   GPIO_PB10,
   GPIO_PB11,
   GPIO_PB12,
   GPIO_PB13,
   GPIO_PB14,
   GPIO_PB15,

   // PORTC pins PC00..PC15
   GPIO_PC00 = 32,
   GPIO_PC01,
   GPIO_PC02,
   GPIO_PC03,
   GPIO_PC04,
   GPIO_PC05,
   GPIO_PC06,
   GPIO_PC07,
   GPIO_PC08,
   GPIO_PC09,
   GPIO_PC10,
   GPIO_PC11,
   GPIO_PC12,
   GPIO_PC13,
   GPIO_PC14,
   GPIO_PC15,

   // DAI0 pin buffers 01..12, 19, 20
   GPIO_DAI0_01 = 48,
   GPIO_DAI0_02,
   GPIO_DAI0_03,
   GPIO_DAI0_04,
   GPIO_DAI0_05,
   GPIO_DAI0_06,
   GPIO_DAI0_07,
   GPIO_DAI0_08,
   GPIO_DAI0_09,
   GPIO_DAI0_10,
   GPIO_DAI0_11,
   GPIO_DAI0_12,
   GPIO_DAI0_19,
   GPIO_DAI0_20,

   // DAI1 pin buffers 01..12, 19, 20
   GPIO_DAI1_01 = 62,
   GPIO_DAI1_02,
   GPIO_DAI1_03,
   GPIO_DAI1_04,
   GPIO_DAI1_05,
   GPIO_DAI1_06,
   GPIO_DAI1_07,
   GPIO_DAI1_08,
   GPIO_DAI1_09,
   GPIO_DAI1_10,
   GPIO_DAI1_11,
   GPIO_DAI1_12,
   GPIO_DAI1_19,
   GPIO_DAI1_20,

   GPIO_PIN_COUNT = 76
};

enum gpio_bank {
   GPIO_BANK_PORTA = 0,
   GPIO_BANK_PORTB,
   GPIO_BANK_PORTC,
   GPIO_BANK_DAI0,
   GPIO_BANK_DAI1,
   GPIO_BANK_COUNT
};

// Configure `pin` as an output. The initial level is low.
//   pin: any value from enum gpio_pin; out-of-range trips an
//        assertion.
void gpio_make_output(const enum gpio_pin pin);

// Configure `pin` as an input. Any previous output drive is
// released.
//   pin: any value from enum gpio_pin; out-of-range trips an
//        assertion.
void gpio_make_input(const enum gpio_pin pin);

// Drive the output level on `pin`. Only meaningful after
// gpio_make_output().
//   pin:  any value from enum gpio_pin; out-of-range trips an
//         assertion.
//   high: true drives high, false drives low.
void gpio_write(const enum gpio_pin pin, const bool high);

// Read the raw status word for a whole GPIO bank.
//   bnk: any value from enum gpio_bank; out-of-range trips an
//        assertion.
//   returns: for PORT banks, 16 bits in [15:0] taken from the
//            pin data register. For DAI banks, the 20-bit
//            DAI_PIN_STAT layout (PB01 in bit 0, gap at bits
//            12..17, PB19/PB20 in bits 18/19).
uint32_t gpio_read_bank(const enum gpio_bank bnk);

#endif // GPIO_H
