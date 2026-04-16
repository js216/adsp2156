// SPDX-License-Identifier: MIT
// mcp23017.c --- Microchip MCP23017 16-bit I/O expander driver
// Copyright (c) 2026 Jakob Kastelic

// The chip comes out of reset in BANK=0 mode, where the A/B
// registers are interleaved at even/odd offsets: IODIRA = 0x00,
// IODIRB = 0x01, GPIOA = 0x12, GPIOB = 0x13 and so on. The
// driver only touches the four registers needed to set pin
// direction and drive the data lines; it leaves the rest at
// their reset defaults.

#include "mcp23017.h"
#include "assert.h"
#include "twi.h"
#include <stdint.h>

#define MCP_IODIRA 0x00U
#define MCP_IODIRB 0x01U
#define MCP_GPIOA  0x12U
#define MCP_GPIOB  0x13U

#define MCP_ADDR7_MASK 0x7FU

int mcp23017_set_direction(const struct mcp23017_target *tgt, uint32_t dir)
{
   ASSERT(tgt->addr <= MCP_ADDR7_MASK);
   ASSERT(tgt->port == MCP23017_PORT_A || tgt->port == MCP23017_PORT_B);
   uint32_t reg = (tgt->port == MCP23017_PORT_A) ? MCP_IODIRA : MCP_IODIRB;
   return twi_write2(tgt->addr, reg, dir);
}

int mcp23017_write_gpio(const struct mcp23017_target *tgt, uint32_t val)
{
   ASSERT(tgt->addr <= MCP_ADDR7_MASK);
   ASSERT(tgt->port == MCP23017_PORT_A || tgt->port == MCP23017_PORT_B);
   uint32_t reg = (tgt->port == MCP23017_PORT_A) ? MCP_GPIOA : MCP_GPIOB;
   return twi_write2(tgt->addr, reg, val);
}
