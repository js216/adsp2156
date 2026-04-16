// SPDX-License-Identifier: MIT
// regs.h --- ADSP-21569 register address map
// Copyright (c) 2026 Jakob Kastelic

// Central register-address header shared by both C sources and
// the assembly startup code. HRM page references are in the
// ADSP-2156x Hardware Reference, PRM references in the SHARC+
// Core Programming Reference. Citation format: chapter-page,
// e.g. HRM 17-26 = chapter 17, page 26.
//
// The C-only block below defines the volatile-pointer MMR
// accessor macros that every driver uses. Assembly files
// include this header too (via -D__ASSEMBLY__ in the build
// flags), so the register-address defines deliberately omit
// the C-only `U` suffix that the assembler cannot parse.

#ifndef REGS_H
#define REGS_H

#ifndef __ASSEMBLY__
#include <stdint.h>
#define MMR(addr)   (*(volatile uint32_t *)(addr))
#define MMR16(addr) (*(volatile uint16_t *)(addr))
#endif

// ---------- SHARC+ core MMRs ----------

#define REG_SHBTB_CFG 0x00031400 // PRM 31-2

// MODE1 sysreg bits (PRM Table 29-2). High bits use explicit
// hex masks because (1U << N) for N >= 31 is signed overflow in
// C and the asm preprocessor can't take the U suffix.
#define BIT_MODE1_NESTM  0x00000800 // PRM 29-50 (NESTM,  bit 11)
#define BIT_MODE1_RND32  0x00010000 // PRM 29-50 (RND32,  bit 16)
#define BIT_MODE1_CBUFEN 0x01000000 // PRM 29-50 (CBUFEN, bit 24)

// ---------- PORTA pinmux (Function Enable + Mux) ----------

#define REG_PORTA_FER 0x31004000 // HRM 12-41
#define REG_PORTA_MUX 0x31004030 // HRM 12-61

// ---------- UART0 ----------

#define REG_UART0_CTL  0x31003004 // HRM 17-26
#define REG_UART0_STAT 0x31003008 // HRM 17-44
#define REG_UART0_CLK  0x31003010 // HRM 17-25
#define REG_UART0_RBR  0x31003020 // HRM 17-40
#define REG_UART0_THR  0x31003024 // HRM 17-50

// UART_CTL bits (Table 17-11, HRM 17-27..17-31).
#define BIT_UART_CTL_EN       (1U << 0U)  // HRM 17-31 (EN,     bit 0)
#define BIT_UART_CTL_MOD_UART (0U << 4U)  // HRM 17-31 (MOD = 0, UART mode)
#define BIT_UART_CTL_WLS_8    (3U << 8U)  // HRM 17-31 (WLS = 3, 8 data bits)
#define BIT_UART_CTL_STB_1    (0U << 12U) // HRM 17-30 (STB = 0, 1 stop bit)
#define BIT_UART_CTL_PEN_OFF  (0U << 14U) // HRM 17-30 (PEN = 0, no parity)

// UART_STAT bits (Table 17-19, HRM 17-44).
#define BIT_UART_STAT_DR   (1U << 0U) // HRM 17-44 (DR,   bit 0)
#define BIT_UART_STAT_THRE (1U << 5U) // HRM 17-44 (THRE, bit 5)

// UART_CLK bits (Table 17-10, HRM 17-25).
#define BIT_UART_CLK_EDBO 0x80000000U // HRM 17-25 (EDBO, bit 31)
#define MASK_UART_CLK_DIV 0x0000FFFFU // HRM 17-25 (DIV, bits 0..15)

// ---------- TWI2 ----------

#define REG_TWI2_CLKDIV   0x31001600 // HRM 21-18
#define REG_TWI2_CTL      0x31001604 // HRM 21-19
#define REG_TWI2_MSTRCTL  0x31001614 // HRM 21-30
#define REG_TWI2_MSTRADDR 0x3100161C // HRM 21-29
#define REG_TWI2_ISTAT    0x31001620 // HRM 21-26
#define REG_TWI2_IMSK     0x31001624 // HRM 21-28
#define REG_TWI2_FIFOCTL  0x31001628 // HRM 21-21
#define REG_TWI2_TXDATA8  0x31001680 // HRM 21-43

// TWI_CTL bits (Table 21-8, HRM 21-19).
#define BIT_TWI_CTL_PRESCALE_M 0x7FU      // HRM 21-19 (PRESCALE, bits 0..6)
#define BIT_TWI_CTL_EN         (1U << 7U) // HRM 21-19 (EN,       bit 7)

// TWI_CLKDIV bit positions (Table 21-7, HRM 21-18).
#define POS_TWI_CLKDIV_CLKLO 0U // HRM 21-18 (CLKLO, bits 0..7)
#define POS_TWI_CLKDIV_CLKHI 8U // HRM 21-18 (CLKHI, bits 8..15)

// TWI_MSTRCTL bits (Table 21-14, HRM 21-30).
#define BIT_TWI_MSTRCTL_EN   (1U << 0U) // HRM 21-30 (EN, bit 0)
#define POS_TWI_MSTRCTL_DCNT 6U         // HRM 21-30 (DCNT, bits 6..13)

// TWI_FIFOCTL bits (Table 21-9, HRM 21-21).
#define BIT_TWI_FIFOCTL_TXFLUSH (1U << 0U) // HRM 21-21 (TXFLUSH, bit 0)
#define BIT_TWI_FIFOCTL_RXFLUSH (1U << 1U) // HRM 21-21 (RXFLUSH, bit 1)

// TWI_ISTAT bits (write-1-to-clear; Table 21-12, HRM 21-26).
#define BIT_TWI_ISTAT_MCOMP  (1U << 4U) // HRM 21-26 (MCOMP,  bit 4)
#define BIT_TWI_ISTAT_MERR   (1U << 5U) // HRM 21-26 (MERR,   bit 5)
#define BIT_TWI_ISTAT_TXSERV (1U << 6U) // HRM 21-26 (TXSERV, bit 6)

// ---------- SPORT top modules ----------
//
// common/sport.c encapsulates every SPORT register address:
// top-module base 0x31002000 + id*0x100 for SPORT0..7, plus
// half-A/B at +0x00 / +0x80. Only the CTL and DIV bitfield
// constants the driver has to cite back to the HRM live here.

// SPORT_CTL_A/B bits (Table 23-25, HRM 23-63..70).
#define BIT_SPORT_CTL_SPENPRI (1U << 0U)  // HRM 23-70 (SPENPRI, bit 0)
#define POS_SPORT_CTL_SLEN    4U          // HRM 23-70 (SLEN,    bits 4..8)
#define BIT_SPORT_CTL_ICLK    (1U << 10U) // HRM 23-69 (ICLK,    bit 10)
#define BIT_SPORT_CTL_CKRE    (1U << 12U) // HRM 23-68 (CKRE,    bit 12)
#define BIT_SPORT_CTL_FSR     (1U << 13U) // HRM 23-68 (FSR,     bit 13)
#define BIT_SPORT_CTL_IFS     (1U << 14U) // HRM 23-68 (IFS,     bit 14)
#define BIT_SPORT_CTL_DIFS    (1U << 15U) // HRM 23-67 (DIFS,    bit 15)
#define BIT_SPORT_CTL_LAFS    (1U << 17U) // HRM 23-67 (LAFS,    bit 17)
#define BIT_SPORT_CTL_SPTRAN  (1U << 25U) // HRM 23-65 (SPTRAN,  bit 25)
#define POS_SPORT_CTL_DXSPRI  30U         // HRM 23-64 (DXSPRI,  bits 30:31)

// SPORT_CTL error status (HRM 23-64).
#define POS_SPORT_CTL_DERRPRI 29U // HRM 23-64 (DERRPRI, bit 29)

// SPORT_DIV bit positions (Table 23-27, HRM 23-80).
#define POS_SPORT_DIV_CLKDIV 0U  // HRM 23-80 (CLKDIV, bits 0..15)
#define POS_SPORT_DIV_FSDIV  16U // HRM 23-80 (FSDIV,  bits 16..31)

// SPORT_ERR bits 4..6 are write-1-to-clear (Table 23-30, HRM 23-82).
#define BIT_SPORT_ERR_DERRP (1U << 4U) // HRM 23-82 (DERRPSTAT, bit 4)
#define BIT_SPORT_ERR_DERRS (1U << 5U) // HRM 23-82 (DERRSSTAT, bit 5)
#define BIT_SPORT_ERR_FSERR (1U << 6U) // HRM 23-82 (FSERRSTAT, bit 6)
#define SPORT_ERR_CLR_ALL                                                      \
   (BIT_SPORT_ERR_DERRP | BIT_SPORT_ERR_DERRS | BIT_SPORT_ERR_FSERR)

// ---------- DAI0 / DAI1 signal routing unit (SRU) ----------
//
// Each DAI unit has its own block of routing registers. The
// group A / C clock and frame-sync registers carry 5-bit
// source codes; group B data registers use 6-bit codes;
// group D pin assignment uses 7-bit codes; group F pin enable
// uses 6-bit codes. DAI_CLK5 / FS4 hold the routing for
// SPT{3,7}_ACLK_I / BCLK_I / AFS_I / BFS_I, which are the
// only SPORT clock and frame-sync destinations in their
// respective DAIs that do not fit in the main CLK0 / FS0
// registers.

#define REG_DAI0_CLK0     0x310C90C0 // HRM 22-55 (group A IN0..5)
#define REG_DAI0_CLK5     0x310C90D4 // HRM 22-60 (group A IN0..1)
#define REG_DAI0_DAT0     0x310C9100 // HRM 22-61 (group B IN0..4)
#define REG_DAI0_DAT1     0x310C9104 // HRM 22-62 (group B IN0..4)
#define REG_DAI0_DAT2     0x310C9108 // HRM 22-63 (group B IN0..4)
#define REG_DAI0_DAT6     0x310C9118 // HRM 22-67 (group B IN0..3)
#define REG_DAI0_FS0      0x310C9140 // HRM 22-68 (group C IN0..5)
#define REG_DAI0_FS4      0x310C9150 // HRM 22-73 (group C IN0..1)
#define REG_DAI0_PIN0     0x310C9180 // HRM 22-74 (group D IN0..3)
#define REG_DAI0_PIN1     0x310C9184 // HRM 22-75 (group D IN0..3)
#define REG_DAI0_PIN2     0x310C9188 // HRM 22-76 (group D IN0..3)
#define REG_DAI0_PIN4     0x310C9190 // HRM 22-78 (group D IN2..5)
#define REG_DAI0_PBEN0    0x310C91E0 // HRM 22-79 (group F IN0..4)
#define REG_DAI0_PBEN1    0x310C91E4 // HRM 22-80 (group F IN0..4)
#define REG_DAI0_PBEN2    0x310C91E8 // HRM 22-81 (group F IN0..1)
#define REG_DAI0_PBEN3    0x310C91EC // HRM 22-82 (group F IN3..4)
#define REG_DAI0_PIN_STAT 0x310C92E4 // HRM 22-115

#define REG_DAI1_CLK0     0x310CA0C0 // DAI1 mirror of DAI0
#define REG_DAI1_CLK5     0x310CA0D4
#define REG_DAI1_DAT0     0x310CA100
#define REG_DAI1_DAT1     0x310CA104
#define REG_DAI1_DAT2     0x310CA108
#define REG_DAI1_DAT6     0x310CA118
#define REG_DAI1_FS0      0x310CA140
#define REG_DAI1_FS4      0x310CA150
#define REG_DAI1_PIN0     0x310CA180
#define REG_DAI1_PIN1     0x310CA184
#define REG_DAI1_PIN2     0x310CA188
#define REG_DAI1_PIN4     0x310CA190
#define REG_DAI1_PBEN0    0x310CA1E0
#define REG_DAI1_PBEN1    0x310CA1E4
#define REG_DAI1_PBEN2    0x310CA1E8
#define REG_DAI1_PBEN3    0x310CA1EC
#define REG_DAI1_PIN_STAT 0x310CA2E4

// PADS0 DAI input-enable registers (HRM 12-119..120). Reset
// value is zero, which isolates every DAI pad -- drivers must
// OR in the all-20-pins mask before any DAI routing takes
// effect.
#define REG_PADS0_DAI0_IE      0x31004460 // HRM 12-119
#define REG_PADS0_DAI1_IE      0x31004464 // HRM 12-120
#define PADS_DAI_ALL_PINS_MASK 0x000FFFFFU

// DAI SRU field-width masks. Group A / C (clock, frame sync)
// fields are 5 bits wide; group B (data) is 6 bits; group D
// (pin assignment) is 7 bits; group F (pin output enable) is
// 6 bits. Drivers building read-modify-write updates use these
// to size the mask they clear before ORing the new code in.
#define BITS_DAI_CLK_FIELD_M  0x1FU
#define BITS_DAI_FS_FIELD_M   0x1FU
#define BITS_DAI_DAT_FIELD_M  0x3FU
#define BITS_DAI_PIN_FIELD_M  0x7FU
#define BITS_DAI_PBEN_FIELD_M 0x3FU

// DAI_PIN2 / DAI_PBEN2 field bit positions for pin buffers
// PB11 and PB12 used by the SPORT3/7 pass-through routing.
#define POS_DAI_PIN2_PB11  14U // HRM 22-76 (group D, PB11)
#define POS_DAI_PIN2_PB12  21U // HRM 22-76 (group D, PB12)
#define POS_DAI_PBEN2_PB11 0U  // HRM 22-81 (group F, PB11)
#define POS_DAI_PBEN2_PB12 6U  // HRM 22-81 (group F, PB12)

// DAI_PIN_STAT bit positions (HRM 22-115). One bit per pin
// buffer; PB13..PB18 don't exist on 21569 and leave a gap in
// the bit layout.
#define BIT_DAI_PIN_STAT_PB01 (1U << 0U)
#define BIT_DAI_PIN_STAT_PB05 (1U << 4U)
#define BIT_DAI_PIN_STAT_PB07 (1U << 6U)
#define BIT_DAI_PIN_STAT_PB08 (1U << 7U)

// Group D and group F static-level source codes (HRM 22-27,
// 22-33). Everything else the SPORT driver needs comes from
// its own per-sport routing table inside sport.c.
#define SRU_D_LOW  0x7EU
#define SRU_D_HIGH 0x7FU
#define SRU_F_LOW  0x00U
#define SRU_F_HIGH 0x01U

// ---------- TIMER0 ----------

#define REG_TIMER0_RUN_SET  0x31018008 // HRM 18-28
#define REG_TIMER0_RUN_CLR  0x3101800C // HRM 18-30
#define REG_TIMER0_TMR0_CFG 0x31018060 // HRM 18-44
#define REG_TIMER0_TMR0_CNT 0x31018064 // HRM 18-49
#define REG_TIMER0_TMR0_PER 0x31018068 // HRM 18-50
#define REG_TIMER0_TMR0_WID 0x3101806C // HRM 18-51
#define REG_TIMER0_TMR0_DLY 0x31018070 // HRM 18-52

// ---------- CGU0 ----------

#define REG_CGU0_CTL  0x3108D000U // HRM 2-18
#define REG_CGU0_STAT 0x3108D008U // HRM 2-24
#define REG_CGU0_DIV  0x3108D00CU // HRM 2-22

#endif // REGS_H
