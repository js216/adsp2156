// SPDX-License-Identifier: MIT
// mcp23017.h --- Microchip MCP23017 16-bit I/O expander driver
// Copyright (c) 2026 Jakob Kastelic

// Thin wrapper over an existing I2C driver that programs the
// MCP23017 direction, pull-up, and data registers. The API is
// stateless: every call takes a target identifying the chip and
// port so a single process can drive multiple MCP23017s on the
// same bus.
//
// The caller is responsible for bringing up the I2C master
// first (twi_init() in this project) and for any board-
// specific pinmux the I2C peripheral might need.

#ifndef MCP23017_H
#define MCP23017_H

#include <stdint.h>

// Port identifier for the chip's two 8-bit I/O ports.
//   MCP23017_PORT_A: bank A (GPIOA, IODIRA, ...)
//   MCP23017_PORT_B: bank B (GPIOB, IODIRB, ...)
enum mcp23017_port {
   MCP23017_PORT_A = 0,
   MCP23017_PORT_B = 1,
};

// Chip + port target for all MCP23017 operations.
//   addr: 7-bit I2C slave address, 0..0x7F.
//   port: MCP23017_PORT_A or MCP23017_PORT_B.
struct mcp23017_target {
   uint32_t addr;
   enum mcp23017_port port;
};

// Program the direction of all 8 pins on the target port.
//   tgt:  chip address and port selection.
//   dir:  bitmask, one bit per pin, 1 = input, 0 = output.
//         (Matches the MCP23017 IODIRx register's native
//         polarity.)
//   returns: 0 on success, -1 on I2C error.
int mcp23017_set_direction(const struct mcp23017_target *tgt, uint32_t dir);

// Drive the output value on the target port. Pins marked as
// outputs in mcp23017_set_direction take the corresponding bit;
// pins marked as inputs ignore the write.
//   tgt:  chip address and port selection.
//   val:  bitmask of output levels, 1 = high, 0 = low.
//   returns: 0 on success, -1 on I2C error.
int mcp23017_write_gpio(const struct mcp23017_target *tgt, uint32_t val);

#endif // MCP23017_H
