// SPDX-License-Identifier: MIT
// twi.h --- Polled TWI2 master-mode driver API
// Copyright (c) 2026 Jakob Kastelic

#ifndef TWI_H
#define TWI_H

#include <stdint.h>

// Clock configuration for twi_init.
//   prescale: internal clock prescaler (SCLK / prescale gives
//             the TWI timebase). Typical value: 12.
//   clkhi:    SCL high-period count in timebase ticks.
//   clklo:    SCL low-period count in timebase ticks.
//             clkhi = clklo = 50 gives ~100 kHz SCL at
//             prescale = 12 and SCLK = 93.75 MHz.
struct twi_clk_cfg {
   uint32_t prescale;
   uint32_t clkhi;
   uint32_t clklo;
};

// Initialise the TWI2 peripheral for master-mode operation.
//   cfg: clock prescale and SCL timing (see struct twi_clk_cfg).
void twi_init(const struct twi_clk_cfg *cfg);

// Master-write two bytes to a 7-bit slave.
//   addr7: 7-bit I2C slave address, 0..0x7F.
//   reg:   first data byte (register index).
//   val:   second data byte (register value).
//   returns: 0 on success, -1 on bus error.
int twi_write2(uint32_t addr7, uint32_t reg, uint32_t val);

#endif // TWI_H
