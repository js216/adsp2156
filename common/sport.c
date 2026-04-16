// SPDX-License-Identifier: MIT
// sport.c --- SHARC+ Serial Port core-polled driver
// Copyright (c) 2026 Jakob Kastelic

// Register layout citations are to chapter 23 of the ADSP-2156x
// Hardware Reference (adsp-2156x_hwr.pdf).

#include "sport.h"
#include "assert.h"
#include "regs.h"
#include <stdbool.h>
#include <stdint.h>

// ----- Register address arithmetic --------------------------

// Top-module bases are 0x100 apart, half B lives 0x80 above
// the module base, half A sits at +0x00. These numbers come
// from the module register list in HRM appendix A.
#define SPORT_MODULE_BASE(id) (0x31002000U + (uint32_t)(id) * 0x100U)
#define SPORT_HALF_BASE(id, h)                                                 \
   (SPORT_MODULE_BASE(id) + ((h) == SPORT_HALF_B ? 0x80U : 0x00U))

// Half-relative register offsets (same layout for A and B).
#define OFF_HALF_CTL   0x00 // HRM 23-63
#define OFF_HALF_DIV   0x04 // HRM 23-80
#define OFF_HALF_MCTL  0x08 // HRM 23-86
#define OFF_HALF_CS0   0x0C
#define OFF_HALF_CS1   0x10
#define OFF_HALF_CS2   0x14
#define OFF_HALF_CS3   0x18
#define OFF_HALF_ERR   0x20 // HRM 23-82
#define OFF_HALF_CTL2  0x28 // HRM 23-61
#define OFF_HALF_TXPRI 0x40 // HRM 23-96
#define OFF_HALF_RXPRI 0x44 // HRM 23-92

static uint32_t half_base(const enum sport_id id, const enum sport_half half)
{
   return SPORT_HALF_BASE(id, half);
}

// Read DXSPRI (bits 31:30 of CTL): 0=empty, 2=partially full,
// 3=full, 1 reserved. The HRM says reads and writes of the
// data buffers update these bits; polling CTL is the sanctioned
// way to check readiness for core-driven transfers.
static uint32_t dxspri(const enum sport_id id, const enum sport_half half)
{
   uint32_t ctl = MMR(half_base(id, half) + OFF_HALF_CTL);
   return (ctl >> POS_SPORT_CTL_DXSPRI) & 0x3U;
}

// ----- Configuration and enable ----------------------------

void sport_dsp_serial_init(const enum sport_id id, const enum sport_half half,
                           const struct sport_dsp_cfg *cfg)
{
   uint32_t base = half_base(id, half);

   // Clear every register the block looks at during startup.
   MMR(base + OFF_HALF_CTL)  = 0U;
   MMR(base + OFF_HALF_CTL2) = 0U;
   MMR(base + OFF_HALF_MCTL) = 0U;
   MMR(base + OFF_HALF_ERR)  = 0U;
   MMR(base + OFF_HALF_CS0)  = 0U;
   MMR(base + OFF_HALF_CS1)  = 0U;
   MMR(base + OFF_HALF_CS2)  = 0U;
   MMR(base + OFF_HALF_CS3)  = 0U;

   MMR(base + OFF_HALF_DIV) = ((uint32_t)(cfg->fsdiv) << POS_SPORT_DIV_FSDIV) |
                              ((uint32_t)cfg->clkdiv << POS_SPORT_DIV_CLKDIV);

   // SLEN is encoded as "word bits - 1" per HRM 23-70.
   uint32_t slen = (cfg->word_bits > 0) ? (cfg->word_bits - 1U) : 0U;

   uint32_t ctl = (slen << POS_SPORT_CTL_SLEN) | BIT_SPORT_CTL_FSR;
   if (cfg->is_tx)
      ctl |= BIT_SPORT_CTL_SPTRAN;
   if (cfg->internal_clk)
      ctl |= BIT_SPORT_CTL_ICLK;
   if (cfg->internal_fs)
      ctl |= BIT_SPORT_CTL_IFS;
   if (cfg->late_fs)
      ctl |= BIT_SPORT_CTL_LAFS;
   if (cfg->data_indep_fs)
      ctl |= BIT_SPORT_CTL_DIFS;
   if (cfg->sample_rising)
      ctl |= BIT_SPORT_CTL_CKRE;

   MMR(base + OFF_HALF_CTL) = ctl;
}

void sport_enable(const enum sport_id id, const enum sport_half half)
{
   uint32_t base = half_base(id, half);
   MMR(base + OFF_HALF_CTL) |= BIT_SPORT_CTL_SPENPRI;
}

void sport_disable(const enum sport_id id, const enum sport_half half)
{
   uint32_t base = half_base(id, half);
   MMR(base + OFF_HALF_CTL) &= ~(uint32_t)BIT_SPORT_CTL_SPENPRI;
}

// ----- Polled data access ----------------------------------

bool sport_rx_ready(const enum sport_id id, const enum sport_half half)
{
   return dxspri(id, half) != 0U; // not empty
}

void sport_write_raw(const enum sport_id id, const enum sport_half half,
                     const uint32_t word)
{
   MMR(half_base(id, half) + OFF_HALF_TXPRI) = word;
}

uint32_t sport_read_raw(const enum sport_id id, const enum sport_half half)
{
   return MMR(half_base(id, half) + OFF_HALF_RXPRI);
}

int sport_read(const enum sport_id id, const enum sport_half half,
               uint32_t *word_out, const uint32_t timeout)
{
   for (uint32_t i = 0; i < timeout; i++) {
      if (sport_rx_ready(id, half)) {
         *word_out = sport_read_raw(id, half);
         return 0;
      }
   }
   return -1;
}

// ----- Internal loopback routing ---------------------------

// Register field descriptor: bit position and unshifted width
// mask. Used by the SRU read-modify-write helper below.
struct reg_field {
   uint32_t pos;
   uint32_t mask;
};

// SRU write helper: replace a masked field at f.pos in a DAI
// routing register with source_code, preserving every other
// field in the same register.
static void sru_set_field(uint32_t reg, const struct reg_field f,
                          uint32_t source_code)
{
   uint32_t v = MMR(reg);
   v &= ~(f.mask << f.pos);
   v |= ((source_code & f.mask) << f.pos);
   MMR(reg) = v;
}

// PADS_DAI[n]_IE bootstrap: the pad cells for each DAI come
// out of reset with their input buffers disabled, so any SRU
// route that touches a DAI pin buffer silently fails until the
// 20-bit mask gets written once. Do it lazily from the
// loopback helper so applications that never use DAI routing
// do not have to know about this bit.
static void enable_dai_pads(uint32_t dai_idx)
{
   uint32_t reg = (dai_idx == 0U) ? (uint32_t)REG_PADS0_DAI0_IE
                                  : (uint32_t)REG_PADS0_DAI1_IE;
   MMR(reg) |= (uint32_t)PADS_DAI_ALL_PINS_MASK;
}

// Per-SPORT routing descriptor. Direct routes (SPORT0..2 and
// SPORT4..6) send the half-A clock, frame sync and data back
// into their own half-B inputs through one DAI_CLK / FS / DAT
// field each. SPORT3 and SPORT7 have no group-A source code
// for their own clock or frame sync, so they use a non-zero
// pin_reg entry: the helper routes SPT{3,7}_ACLK_O to
// DAI_PIN2.PB11 via group D and enables PB11 as an output,
// then routes DAI_PB11_O into SPT{3,7}_BCLK_I through
// DAI_CLK5.IN1 (and the matching pair for frame sync on
// PB12 / DAI_FS4).
struct sport_route {
   uint32_t clk_reg; // DAI_CLKx register address
   uint32_t clk_bit; // destination field bit position
   uint32_t clk_src; // group-A source code
   uint32_t fs_reg;
   uint32_t fs_bit;
   uint32_t fs_src;
   uint32_t dat_reg; // DAI_DATx register address
   uint32_t dat_bit;
   uint32_t dat_src; // group-B source code
   // Pin-buffer pass-through, SPORT3 / SPORT7 only. pin_reg
   // is the DAI_PIN2 register; pben_reg is DAI_PBEN2; the
   // source codes go at bit positions 14 (PB11, clock) and
   // 21 (PB12, frame sync). pin_reg == 0 means "no
   // pass-through, use direct SRU routing instead".
   uint32_t pin_reg;
   uint32_t pben_reg;
   uint32_t clk_pin_src_d; // group-D source code for SPT*_ACLK_O
   uint32_t fs_pin_src_d;  // group-D source code for SPT*_AFS_O
};

static const struct sport_route sport_routes[SPORT_ID_COUNT] = {
    // SPORT0..2 on DAI0: CLK0 / FS0 IN1/3/5, DAT0/1/2 IN2/1/0.
    // Group-A source codes 0x14/0x16/0x18 come from HRM
    // Table 22-10; group-B 0x14/0x18/0x1C from Table 22-11;
    // group-C 0x14/0x16/0x18 from Table 22-12.
    {REG_DAI0_CLK0, 5U,  0x14U, REG_DAI0_FS0, 5U,  0x14U, REG_DAI0_DAT0, 12U,
     0x14U, 0U,            0U,             0U,    0U   },
    {REG_DAI0_CLK0, 15U, 0x16U, REG_DAI0_FS0, 15U, 0x16U, REG_DAI0_DAT1, 6U,
     0x18U, 0U,            0U,             0U,    0U   },
    {REG_DAI0_CLK0, 25U, 0x18U, REG_DAI0_FS0, 25U, 0x18U, REG_DAI0_DAT2, 0U,
     0x1CU, 0U,            0U,             0U,    0U   },
    // SPORT3: CLK5 / FS4 IN1 with DAI0_PB11_O / PB12_O as the
    // group-A / C sources; DAT6 IN2 with SPT3_AD0_O (0x2C).
    // Pin-buffer pass-through uses group-D codes 0x34 / 0x36
    // for SPT3_ACLK_O / AFS_O (HRM Table 22-13).
    {REG_DAI0_CLK5, 5U,  0x0AU, REG_DAI0_FS4, 5U,  0x0BU, REG_DAI0_DAT6, 12U,
     0x2CU, REG_DAI0_PIN2, REG_DAI0_PBEN2, 0x34U, 0x36U},
    // SPORT4..6 on DAI1: mirror of SPORT0..2.
    {REG_DAI1_CLK0, 5U,  0x14U, REG_DAI1_FS0, 5U,  0x14U, REG_DAI1_DAT0, 12U,
     0x14U, 0U,            0U,             0U,    0U   },
    {REG_DAI1_CLK0, 15U, 0x16U, REG_DAI1_FS0, 15U, 0x16U, REG_DAI1_DAT1, 6U,
     0x18U, 0U,            0U,             0U,    0U   },
    {REG_DAI1_CLK0, 25U, 0x18U, REG_DAI1_FS0, 25U, 0x18U, REG_DAI1_DAT2, 0U,
     0x1CU, 0U,            0U,             0U,    0U   },
    // SPORT7: mirror of SPORT3 on DAI1.
    {REG_DAI1_CLK5, 5U,  0x0AU, REG_DAI1_FS4, 5U,  0x0BU, REG_DAI1_DAT6, 12U,
     0x2CU, REG_DAI1_PIN2, REG_DAI1_PBEN2, 0x34U, 0x36U},
};

void sport_install_internal_loopback(const enum sport_id id)
{
   ASSERT((uint32_t)id < SPORT_ID_COUNT);
   const struct sport_route *r = &sport_routes[id];

   uint32_t dai_idx = ((uint32_t)id < 4U) ? 0U : 1U;
   enable_dai_pads(dai_idx);

   if (r->pin_reg != 0U) {
      // Pin-buffer pass-through for SPORT3 / SPORT7: drive
      // ACLK_O and AFS_O out on PB11 / PB12 so their group-A
      // / C aliases become valid SRU sources.
      sru_set_field(r->pin_reg,
                    (struct reg_field){POS_DAI_PIN2_PB11, BITS_DAI_PIN_FIELD_M},
                    r->clk_pin_src_d);
      sru_set_field(r->pin_reg,
                    (struct reg_field){POS_DAI_PIN2_PB12, BITS_DAI_PIN_FIELD_M},
                    r->fs_pin_src_d);
      sru_set_field(
          r->pben_reg,
          (struct reg_field){POS_DAI_PBEN2_PB11, BITS_DAI_PBEN_FIELD_M},
          SRU_F_HIGH);
      sru_set_field(
          r->pben_reg,
          (struct reg_field){POS_DAI_PBEN2_PB12, BITS_DAI_PBEN_FIELD_M},
          SRU_F_HIGH);
   }
   sru_set_field(r->clk_reg,
                 (struct reg_field){r->clk_bit, BITS_DAI_CLK_FIELD_M},
                 r->clk_src);
   sru_set_field(r->fs_reg, (struct reg_field){r->fs_bit, BITS_DAI_FS_FIELD_M},
                 r->fs_src);
   sru_set_field(r->dat_reg,
                 (struct reg_field){r->dat_bit, BITS_DAI_DAT_FIELD_M},
                 r->dat_src);
}

void sport_clear_errors(const enum sport_id id, const enum sport_half half)
{
   MMR(half_base(id, half) + OFF_HALF_ERR) = SPORT_ERR_CLR_ALL;
}

bool sport_has_error(const enum sport_id id, const enum sport_half half)
{
   uint32_t ctl = MMR(half_base(id, half) + OFF_HALF_CTL);
   return (ctl & (1U << POS_SPORT_CTL_DERRPRI)) != 0U;
}

// Default half-A pin-buffer enable mapping. Each SPORT module
// uses 3 pins for its half-A TX side: AD0 (data), ACLK
// (clock), AFS (frame sync). The PBEN register and field bit
// positions come from the reset-default pin assignment
// (HRM 22-79, Table 22-26).
struct sport_pin_enable {
   uint32_t pben_reg; // DAI_PBEN register address
   uint32_t ad0_bit;  // bit position of the AD0 PBEN field
   uint32_t aclk_bit; // bit position of the ACLK PBEN field
   uint32_t afs_bit;  // bit position of the AFS PBEN field
};

static const struct sport_pin_enable pin_enables[SPORT_ID_COUNT] = {
    // DAI0 SPORT0..3.
    // SPORT0: PB01(AD0)=PBEN0 bit 0, PB03(ACLK)=PBEN0 bit 12, PB04(AFS)=PBEN0
    // bit 18
    {REG_DAI0_PBEN0, 0U, 12U, 18U},
    // SPORT1: PB05(AD0)=PBEN0 bit 24, PB07(ACLK)=PBEN1 bit 6, PB08(AFS)=PBEN1
    // bit 12
    {0U,             0U, 0U,  0U }, // SPORT1 spans two PBEN regs -- not yet supported
    // SPORT2: similar complexity
    {0U,             0U, 0U,  0U },
    // SPORT3: PB19/PB20 only (2 data, no dedicated CLK/FS pin)
    {0U,             0U, 0U,  0U },
    // DAI1 SPORT4..7 (same layout as 0..3 but on DAI1).
    {REG_DAI1_PBEN0, 0U, 12U, 18U},
    {0U,             0U, 0U,  0U },
    {0U,             0U, 0U,  0U },
    {0U,             0U, 0U,  0U },
};

void sport_enable_external_pins(const enum sport_id id)
{
   ASSERT((uint32_t)id < SPORT_ID_COUNT);
   const struct sport_pin_enable *pe = &pin_enables[id];
   ASSERT(pe->pben_reg != 0U);

   uint32_t dai_idx = ((uint32_t)id < 4U) ? 0U : 1U;
   enable_dai_pads(dai_idx);

   uint32_t mask = (uint32_t)BITS_DAI_PBEN_FIELD_M;
   uint32_t v    = MMR(pe->pben_reg);
   v &= ~((mask << pe->ad0_bit) | (mask << pe->aclk_bit) |
          (mask << pe->afs_bit));
   v |= ((uint32_t)SRU_F_HIGH << pe->ad0_bit) |
        ((uint32_t)SRU_F_HIGH << pe->aclk_bit) |
        ((uint32_t)SRU_F_HIGH << pe->afs_bit);
   MMR(pe->pben_reg) = v;
}
