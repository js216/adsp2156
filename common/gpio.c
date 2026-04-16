// SPDX-License-Identifier: MIT
// gpio.c --- General-purpose GPIO driver (PORT + DAI pin buffers)
// Copyright (c) 2026 Jakob Kastelic

#include "gpio.h"
#include "assert.h"
#include "regs.h"
#include <stdbool.h>
#include <stdint.h>

// PORT block per-pin MMR offsets (HRM 12-28..12-47). FER is
// the function-enable (clear for GPIO mode), DIR the output
// direction, DATA the driven / sampled data, INEN the input
// buffer enable. _SET / _CLR are the write-to-one-to-set and
// write-to-one-to-clear aliases at 0x10 / 0x14, 0x1C / 0x20
// and 0x28 / 0x2C.
#define OFF_PORT_FER      0x00 // HRM 12-28
#define OFF_PORT_FER_CLR  0x08 // HRM 12-30
#define OFF_PORT_DATA     0x0C // HRM 12-32
#define OFF_PORT_DATA_SET 0x10 // HRM 12-33
#define OFF_PORT_DATA_CLR 0x14 // HRM 12-34
#define OFF_PORT_DIR_SET  0x1C // HRM 12-38
#define OFF_PORT_DIR_CLR  0x20 // HRM 12-39
#define OFF_PORT_INEN_SET 0x28 // HRM 12-42
#define OFF_PORT_INEN_CLR 0x2C // HRM 12-43

// The three PORT blocks live 0x80 apart starting at PORTA
// (HRM 12-27). Bases come from the register list in appendix A.
static const uint32_t port_base[3] = {
    0x31004000U, // PORTA
    0x31004080U, // PORTB
    0x31004100U, // PORTC
};

// DAI pin assignment / pin-buffer enable / pin status
// register offsets within a DAI block (HRM 22-107..22-115).
#define OFF_DAI_PIN0     0x180 // HRM 22-111
#define OFF_DAI_PIN1     0x184 // HRM 22-112
#define OFF_DAI_PIN2     0x188 // HRM 22-113
#define OFF_DAI_PIN4     0x190 // HRM 22-114
#define OFF_DAI_PBEN0    0x1E0 // HRM 22-107
#define OFF_DAI_PBEN1    0x1E4 // HRM 22-108
#define OFF_DAI_PBEN2    0x1E8 // HRM 22-109
#define OFF_DAI_PBEN3    0x1EC // HRM 22-110
#define OFF_DAI_PIN_STAT 0x2E4 // HRM 22-115

// DAI top-level bases (HRM 22-107, appendix A).
static const uint32_t dai_base[2] = {
    0x310C9000U, // DAI0
    0x310CA000U, // DAI1
};

// PADS_DAI[01]_IE input-enable registers live in the PADS
// block, not in the DAI block. Their reset value is zero,
// which disables every DAI pad's input buffer and cuts off
// the whole pad cell. This helper ORs the full 20-bit mask
// in; writing the same bits repeatedly is harmless (OR is
// idempotent). sport.c has an identical helper for the same
// reason -- both call sites are independent and either may
// run first.
static const uint32_t pads_dai_ie[2] = {
    REG_PADS0_DAI0_IE,
    REG_PADS0_DAI1_IE,
};

static void dai_pads_enable(uint32_t unit)
{
   MMR(pads_dai_ie[unit]) |= PADS_DAI_ALL_PINS_MASK;
}

// ----- Per-pin descriptor tables ----------------------------

#define KIND_PORT 0
#define KIND_DAI  1

struct pin_info {
   uint32_t kind; // KIND_PORT or KIND_DAI
   uint32_t bnk;  // 0..2 for PORTA/B/C, 0..1 for DAI0/DAI1
   // For KIND_PORT: bit number within the 16-bit PORT
   // registers (0..15). d_off/d_bit/f_off/f_bit unused.
   //
   // For KIND_DAI: bit number in DAI_PIN_STAT (0..19),
   // plus the register offsets and bit positions of the
   // pin's group-D (PIN) and group-F (PBEN) fields.
   // Fields are 32-bit rather than 8-bit because cc21k with
   // -char-size-8 still word-aligns every field in a struct
   // regardless, and specifying uint32_t makes the
   // aggregate-initializer expansions land in the expected
   // fields.
   uint32_t stat_bit;
   uint32_t d_off;
   uint32_t d_bit;
   uint32_t f_off;
   uint32_t f_bit;
};

// Initializer helpers to keep the big table below readable.
#define PORT_PIN(bnk, bit) {KIND_PORT, (bnk), (bit), 0, 0, 0, 0}
#define DAI_PIN(bnk, sb, dreg, dbit, freg, fbit)                               \
   {KIND_DAI, (bnk), (sb), (dreg), (dbit), (freg), (fbit)}

static const struct pin_info pin_info[GPIO_PIN_COUNT] = {
    // PORTA (bank 0)
    PORT_PIN(0, 0), PORT_PIN(0, 1), PORT_PIN(0, 2), PORT_PIN(0, 3),
    PORT_PIN(0, 4), PORT_PIN(0, 5), PORT_PIN(0, 6), PORT_PIN(0, 7),
    PORT_PIN(0, 8), PORT_PIN(0, 9), PORT_PIN(0, 10), PORT_PIN(0, 11),
    PORT_PIN(0, 12), PORT_PIN(0, 13), PORT_PIN(0, 14), PORT_PIN(0, 15),
    // PORTB (bank 1)
    PORT_PIN(1, 0), PORT_PIN(1, 1), PORT_PIN(1, 2), PORT_PIN(1, 3),
    PORT_PIN(1, 4), PORT_PIN(1, 5), PORT_PIN(1, 6), PORT_PIN(1, 7),
    PORT_PIN(1, 8), PORT_PIN(1, 9), PORT_PIN(1, 10), PORT_PIN(1, 11),
    PORT_PIN(1, 12), PORT_PIN(1, 13), PORT_PIN(1, 14), PORT_PIN(1, 15),
    // PORTC (bank 2)
    PORT_PIN(2, 0), PORT_PIN(2, 1), PORT_PIN(2, 2), PORT_PIN(2, 3),
    PORT_PIN(2, 4), PORT_PIN(2, 5), PORT_PIN(2, 6), PORT_PIN(2, 7),
    PORT_PIN(2, 8), PORT_PIN(2, 9), PORT_PIN(2, 10), PORT_PIN(2, 11),
    PORT_PIN(2, 12), PORT_PIN(2, 13), PORT_PIN(2, 14), PORT_PIN(2, 15),
    // DAI0 (bank 0) pin buffers 01..12 and 19..20
    DAI_PIN(0, 0, OFF_DAI_PIN0, 0, OFF_DAI_PBEN0, 0),    // DAI0 PB01
    DAI_PIN(0, 1, OFF_DAI_PIN0, 7, OFF_DAI_PBEN0, 6),    // DAI0 PB02
    DAI_PIN(0, 2, OFF_DAI_PIN0, 14, OFF_DAI_PBEN0, 12),  // DAI0 PB03
    DAI_PIN(0, 3, OFF_DAI_PIN0, 21, OFF_DAI_PBEN0, 18),  // DAI0 PB04
    DAI_PIN(0, 4, OFF_DAI_PIN1, 0, OFF_DAI_PBEN0, 24),   // DAI0 PB05
    DAI_PIN(0, 5, OFF_DAI_PIN1, 7, OFF_DAI_PBEN1, 0),    // DAI0 PB06
    DAI_PIN(0, 6, OFF_DAI_PIN1, 14, OFF_DAI_PBEN1, 6),   // DAI0 PB07
    DAI_PIN(0, 7, OFF_DAI_PIN1, 21, OFF_DAI_PBEN1, 12),  // DAI0 PB08
    DAI_PIN(0, 8, OFF_DAI_PIN2, 0, OFF_DAI_PBEN1, 18),   // DAI0 PB09
    DAI_PIN(0, 9, OFF_DAI_PIN2, 7, OFF_DAI_PBEN1, 24),   // DAI0 PB10
    DAI_PIN(0, 10, OFF_DAI_PIN2, 14, OFF_DAI_PBEN2, 0),  // DAI0 PB11
    DAI_PIN(0, 11, OFF_DAI_PIN2, 21, OFF_DAI_PBEN2, 6),  // DAI0 PB12
    DAI_PIN(0, 18, OFF_DAI_PIN4, 14, OFF_DAI_PBEN3, 18), // DAI0 PB19
    DAI_PIN(0, 19, OFF_DAI_PIN4, 21, OFF_DAI_PBEN3, 24), // DAI0 PB20
    // DAI1 (bank 1) pin buffers 01..12 and 19..20
    DAI_PIN(1, 0, OFF_DAI_PIN0, 0, OFF_DAI_PBEN0, 0),    // DAI1 PB01
    DAI_PIN(1, 1, OFF_DAI_PIN0, 7, OFF_DAI_PBEN0, 6),    // DAI1 PB02
    DAI_PIN(1, 2, OFF_DAI_PIN0, 14, OFF_DAI_PBEN0, 12),  // DAI1 PB03
    DAI_PIN(1, 3, OFF_DAI_PIN0, 21, OFF_DAI_PBEN0, 18),  // DAI1 PB04
    DAI_PIN(1, 4, OFF_DAI_PIN1, 0, OFF_DAI_PBEN0, 24),   // DAI1 PB05
    DAI_PIN(1, 5, OFF_DAI_PIN1, 7, OFF_DAI_PBEN1, 0),    // DAI1 PB06
    DAI_PIN(1, 6, OFF_DAI_PIN1, 14, OFF_DAI_PBEN1, 6),   // DAI1 PB07
    DAI_PIN(1, 7, OFF_DAI_PIN1, 21, OFF_DAI_PBEN1, 12),  // DAI1 PB08
    DAI_PIN(1, 8, OFF_DAI_PIN2, 0, OFF_DAI_PBEN1, 18),   // DAI1 PB09
    DAI_PIN(1, 9, OFF_DAI_PIN2, 7, OFF_DAI_PBEN1, 24),   // DAI1 PB10
    DAI_PIN(1, 10, OFF_DAI_PIN2, 14, OFF_DAI_PBEN2, 0),  // DAI1 PB11
    DAI_PIN(1, 11, OFF_DAI_PIN2, 21, OFF_DAI_PBEN2, 6),  // DAI1 PB12
    DAI_PIN(1, 18, OFF_DAI_PIN4, 14, OFF_DAI_PBEN3, 18), // DAI1 PB19
    DAI_PIN(1, 19, OFF_DAI_PIN4, 21, OFF_DAI_PBEN3, 24), // DAI1 PB20
};

// Register field descriptor: bit position and unshifted width
// mask. Used by the read-modify-write helper below.
struct reg_field {
   uint32_t pos;
   uint32_t mask;
};

// Read-modify-write helper: replace a masked field at f.pos
// with `val` inside the MMR at `addr`.
static void mmr_field_set(uint32_t addr, const struct reg_field f, uint32_t val)
{
   uint32_t v = MMR(addr);
   v &= ~(f.mask << f.pos);
   v |= ((val & f.mask) << f.pos);
   MMR(addr) = v;
}

// Return a pointer to the descriptor for `pin`. Fails an
// assertion on out-of-range input so a bad call is caught at
// the first offending line instead of silently mis-routing.
static const struct pin_info *pin_lookup(const enum gpio_pin pin)
{
   ASSERT((uint32_t)pin < GPIO_PIN_COUNT);
   return &pin_info[pin];
}

void gpio_make_output(const enum gpio_pin pin)
{
   const struct pin_info *p = pin_lookup(pin);
   if (p->kind == KIND_PORT) {
      uint32_t base = port_base[p->bnk];
      uint32_t mask = 1U << p->stat_bit;
      // Take the pin out of peripheral mode, drive 0, then
      // assert direction = output.
      MMR(base + OFF_PORT_FER_CLR)  = mask;
      MMR(base + OFF_PORT_DATA_CLR) = mask;
      MMR(base + OFF_PORT_INEN_CLR) = mask;
      MMR(base + OFF_PORT_DIR_SET)  = mask;
   } else {
      dai_pads_enable(p->bnk);
      uint32_t base = dai_base[p->bnk];
      // Group-D source = LOW so the pin starts driving 0;
      // group-F enable = HIGH turns the output driver on.
      mmr_field_set(base + p->d_off,
                    (struct reg_field){p->d_bit, BITS_DAI_PIN_FIELD_M},
                    SRU_D_LOW);
      mmr_field_set(base + p->f_off,
                    (struct reg_field){p->f_bit, BITS_DAI_PBEN_FIELD_M},
                    SRU_F_HIGH);
   }
}

void gpio_make_input(const enum gpio_pin pin)
{
   const struct pin_info *p = pin_lookup(pin);
   if (p->kind == KIND_PORT) {
      uint32_t base                 = port_base[p->bnk];
      uint32_t mask                 = 1U << p->stat_bit;
      MMR(base + OFF_PORT_FER_CLR)  = mask;
      MMR(base + OFF_PORT_DIR_CLR)  = mask;
      MMR(base + OFF_PORT_INEN_SET) = mask;
   } else {
      dai_pads_enable(p->bnk);
      uint32_t base = dai_base[p->bnk];
      // Group-F enable = LOW tri-states the output driver.
      mmr_field_set(base + p->f_off,
                    (struct reg_field){p->f_bit, BITS_DAI_PBEN_FIELD_M},
                    SRU_F_LOW);
   }
}

void gpio_write(const enum gpio_pin pin, const bool high)
{
   const struct pin_info *p = pin_lookup(pin);
   if (p->kind == KIND_PORT) {
      uint32_t base = port_base[p->bnk];
      uint32_t mask = 1U << p->stat_bit;
      if (high) {
         MMR(base + OFF_PORT_DATA_SET) = mask;
      } else {
         MMR(base + OFF_PORT_DATA_CLR) = mask;
      }
   } else {
      uint32_t base = dai_base[p->bnk];
      mmr_field_set(base + p->d_off,
                    (struct reg_field){p->d_bit, BITS_DAI_PIN_FIELD_M},
                    high ? SRU_D_HIGH : SRU_D_LOW);
   }
}

uint32_t gpio_read_bank(const enum gpio_bank bnk)
{
   ASSERT((uint32_t)bnk < GPIO_BANK_COUNT);
   switch (bnk) {
      case GPIO_BANK_PORTA: return MMR(port_base[0] + OFF_PORT_DATA);
      case GPIO_BANK_PORTB: return MMR(port_base[1] + OFF_PORT_DATA);
      case GPIO_BANK_PORTC: return MMR(port_base[2] + OFF_PORT_DATA);
      case GPIO_BANK_DAI0: return MMR(dai_base[0] + OFF_DAI_PIN_STAT);
      case GPIO_BANK_DAI1: return MMR(dai_base[1] + OFF_DAI_PIN_STAT);
      default:
         // Unreachable (assertion above).
         return 0;
   }
}
