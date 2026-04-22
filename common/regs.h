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

#define REG_PORTA_FER      0x31004000U // HRM 12-41
#define REG_PORTA_DATA     0x3100400CU // HRM 12-46
#define REG_PORTA_INEN     0x31004024U // HRM 12-49
#define REG_PORTA_INEN_SET 0x31004028U // HRM 12-49
#define REG_PORTA_MUX      0x31004030U // HRM 12-61

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

// ---------- DPM0 (power management) ----------

#define REG_DPM0_PER_DIS0 0x31090070U // HRM 4-7

// ---------- FIR0 accelerator ----------

#define REG_FIR0_CTL1    0x310C3000U // HRM 38-27
#define REG_FIR0_DMASTAT 0x310C3004U // HRM 38-36
#define REG_FIR0_MACSTAT 0x310C3008U // HRM 38-42
#define REG_FIR0_CTL2    0x310C3040U // HRM 38-30
#define REG_FIR0_INIDX   0x310C3044U // HRM 38-40
#define REG_FIR0_INMOD   0x310C3048U // HRM 38-41
#define REG_FIR0_INCNT   0x310C304CU // HRM 38-39
#define REG_FIR0_INBASE  0x310C3050U // HRM 38-38
#define REG_FIR0_OUTIDX  0x310C3054U // HRM 38-47
#define REG_FIR0_OUTMOD  0x310C3058U // HRM 38-48
#define REG_FIR0_OUTCNT  0x310C305CU // HRM 38-46
#define REG_FIR0_OUTBASE 0x310C3060U // HRM 38-45
#define REG_FIR0_COEFIDX 0x310C3064U // HRM 38-25
#define REG_FIR0_COEFMOD 0x310C3068U // HRM 38-26
#define REG_FIR0_COEFCNT 0x310C306CU // HRM 38-24
#define REG_FIR0_CHNPTR  0x310C3070U // HRM 38-23

// FIR_CTL1 bits (Table 38-11, HRM 38-27).
#define BIT_FIR_CTL1_EN      (1U << 0U) // HRM 38-29 (EN,      bit 0)
#define BIT_FIR_CTL1_DMAEN   (1U << 8U) // HRM 38-28 (DMAEN,   bit 8)
#define BIT_FIR_CTL1_BURSTEN (1U << 6U) // HRM 38-29 (BURSTEN, bit 6)
#define POS_FIR_CTL1_CH      1U         // HRM 38-29 (CH,      bits 1..5)
#define MASK_FIR_CTL1_CH     0x3EU      // 5-bit channel field

// FIR_CTL2 bits (Table 38-12, HRM 38-30).
#define POS_FIR_CTL2_TAPLEN  0U // HRM 38-31 (TAPLEN, bits 0..11)
#define MASK_FIR_CTL2_TAPLEN 0xFFFU
#define POS_FIR_CTL2_WINDOW  14U       // HRM 38-31 (WINDOW, bits 14..23)
#define MASK_FIR_CTL2_WINDOW 0x3FF000U // 10 bits already shifted

// FIR_DMASTAT bits (Table 38-15, HRM 38-36).
#define BIT_FIR_DMASTAT_ACDONE (1U << 6U) // HRM 38-37 (ACDONE, bit 6)
#define BIT_FIR_DMASTAT_WDONE  (1U << 5U) // HRM 38-37 (WDONE,  bit 5)

// ---------- IIR0 accelerator ----------

#define REG_IIR0_CTL1    0x310C4000U // HRM 39-18
#define REG_IIR0_DMASTAT 0x310C4004U // HRM 39-24
#define REG_IIR0_MACSTAT 0x310C4008U // HRM 39-28
#define REG_IIR0_CTL2    0x310C4040U // HRM 39-21
#define REG_IIR0_INIDX   0x310C4044U // HRM 39-30
#define REG_IIR0_INMOD   0x310C4048U // HRM 39-31
#define REG_IIR0_INLEN   0x310C404CU // HRM 39-29
#define REG_IIR0_INBASE  0x310C4050U // HRM 39-27
#define REG_IIR0_OUTIDX  0x310C4054U // HRM 39-35
#define REG_IIR0_OUTMOD  0x310C4058U // HRM 39-36
#define REG_IIR0_OUTLEN  0x310C405CU // HRM 39-34
#define REG_IIR0_OUTBASE 0x310C4060U // HRM 39-33
#define REG_IIR0_COEFIDX 0x310C4064U // HRM 39-16
#define REG_IIR0_COEFMOD 0x310C4068U // HRM 39-17
#define REG_IIR0_COEFLEN 0x310C406CU // HRM 39-15
#define REG_IIR0_CHNPTR  0x310C4070U // HRM 39-14

// IIR_CTL1 bits (Table 39-6, HRM 39-18).
#define BIT_IIR_CTL1_EN      (1U << 0U)  // HRM 39-20 (EN,      bit 0)
#define BIT_IIR_CTL1_DMAEN   (1U << 8U)  // HRM 39-19 (DMAEN,   bit 8)
#define BIT_IIR_CTL1_SS      (1U << 10U) // HRM 39-19 (SS,      bit 10)
#define BIT_IIR_CTL1_BURSTEN (1U << 17U) // HRM 39-19 (BURSTEN, bit 17)
#define POS_IIR_CTL1_CH      1U          // HRM 39-20 (CH,      bits 1..5)

// IIR_CTL2 bits (Table 39-7, HRM 39-21).
#define POS_IIR_CTL2_BIQUADS 0U  // HRM 39-22 (BIQUADS, bits 0..5)
#define POS_IIR_CTL2_WINDOW  14U // HRM 39-22 (WINDOW,  bits 14..23)

// IIR_DMASTAT bits (Table 39-9, HRM 39-24).
#define BIT_IIR_DMASTAT_ACDONE (1U << 6U) // HRM 39-25 (ACDONE, bit 6)
#define BIT_IIR_DMASTAT_WDONE  (1U << 5U) // HRM 39-25 (WDONE,  bit 5)

// ---------- SPI ----------
//
// The ADSP-2156x has three SPI modules (SPI0..2). SPI2 supports
// quad I/O. Each module's registers sit at a 0x1000-byte stride
// starting from SPI0's base. The driver computes per-instance
// addresses internally; only the SPI_CTL / SPI_TXCTL / SPI_RXCTL
// / SPI_STAT bitfield constants live here.

#define REG_SPI0_BASE 0x3102E000U // HRM 15-2
#define REG_SPI1_BASE 0x3102F000U // HRM 15-2
#define REG_SPI2_BASE 0x31030000U // HRM 15-2

// SCB5 REMAP selects whether the shared flash/pin bus is owned
// by SPI2 (REMAP = 0) or by SPI3/OSPI (REMAP = 1). The boot ROM
// may leave it pointing at OSPI, which silently diverts SPI2
// pin traffic; the ADI reference driver clears it unconditionally
// when bringing up SPI2. ADI ADSP-2156x HPC header gives this
// the symbol REG_SCB5_REMAP.
#define REG_SCB5_REMAP 0x30400000U

// OSPI0 (SPI3) shares the SPI2/OSPI pin group on PA0..PA5 with
// SPI2. The boot ROM may leave OSPI0 enabled so that it contends
// with SPI2 for the shared pin bus; clear OSPI0_CTL.EN before
// bringing SPI2 up. See HRM chapter 16 (OSPI) for the register
// layout. Only bit 0 (EN) matters here.
#define REG_OSPI0_CTL   0x31027000U
#define BIT_OSPI_CTL_EN (1U << 0U)

// Per-module register offsets (Table 15-2, HRM 15-2).
#define OFF_SPI_CTL      0x04U // HRM 15-36
#define OFF_SPI_RXCTL    0x08U // HRM 15-64
#define OFF_SPI_TXCTL    0x0CU // HRM 15-78
#define OFF_SPI_CLK      0x10U // HRM 15-35
#define OFF_SPI_DLY      0x14U // HRM 15-42
#define OFF_SPI_SLVSEL   0x18U // HRM 15-67
#define OFF_SPI_RWC      0x1CU // HRM 15-62
#define OFF_SPI_RWCR     0x20U // HRM 15-63
#define OFF_SPI_TWC      0x24U // HRM 15-76
#define OFF_SPI_TWCR     0x28U // HRM 15-77
#define OFF_SPI_STAT     0x40U // HRM 15-70
#define OFF_SPI_ILAT_CLR 0x48U // HRM 15-45
#define OFF_SPI_RFIFO    0x50U // HRM 15-61
#define OFF_SPI_TFIFO    0x58U // HRM 15-75

// SPI_CTL bits (Table 15-14, HRM 15-36).
#define BIT_SPI_CTL_EN    (1U << 0U)  // HRM 15-41 (EN,     bit 0)
#define BIT_SPI_CTL_MSTR  (1U << 1U)  // HRM 15-41 (MSTR,   bit 1)
#define BIT_SPI_CTL_PSSE  (1U << 2U)  // HRM 15-41 (PSSE,   bit 2)
#define BIT_SPI_CTL_ODM   (1U << 3U)  // HRM 15-41 (ODM,    bit 3)
#define BIT_SPI_CTL_CPHA  (1U << 4U)  // HRM 15-40 (CPHA,   bit 4)
#define BIT_SPI_CTL_CPOL  (1U << 5U)  // HRM 15-40 (CPOL,   bit 5)
#define BIT_SPI_CTL_ASSEL (1U << 6U)  // HRM 15-40 (ASSEL,  bit 6)
#define BIT_SPI_CTL_SELST (1U << 7U)  // HRM 15-40 (SELST,  bit 7)
#define BIT_SPI_CTL_EMISO (1U << 8U)  // HRM 15-40 (EMISO,  bit 8)
#define POS_SPI_CTL_SIZE  9U          // HRM 15-40 (SIZE,   bits 9..10)
#define BIT_SPI_CTL_LSBF  (1U << 12U) // HRM 15-39 (LSBF,   bit 12)
#define BIT_SPI_CTL_FMODE (1U << 18U) // HRM 15-38 (FMODE,  bit 18)
#define POS_SPI_CTL_MIOM  20U         // HRM 15-38 (MIOM,   bits 20..21)
#define BIT_SPI_CTL_SOSI  (1U << 22U) // HRM 15-37 (SOSI,   bit 22)

// SPI_CTL SIZE field values (HRM 15-40).
#define SPI_SIZE_8  (0U << POS_SPI_CTL_SIZE)
#define SPI_SIZE_16 (1U << POS_SPI_CTL_SIZE)
#define SPI_SIZE_32 (2U << POS_SPI_CTL_SIZE)

// SPI_CTL MIOM field values (HRM 15-38).
#define SPI_MIOM_DIS  (0U << POS_SPI_CTL_MIOM) // single I/O
#define SPI_MIOM_DUAL (1U << POS_SPI_CTL_MIOM) // dual I/O
#define SPI_MIOM_QUAD (2U << POS_SPI_CTL_MIOM) // quad I/O (SPI2)

// SPI_RXCTL bits (Table 15-23, HRM 15-64).
#define BIT_SPI_RXCTL_REN   (1U << 0U) // HRM 15-66 (REN,   bit 0)
#define BIT_SPI_RXCTL_RTI   (1U << 2U) // HRM 15-66 (RTI,   bit 2)
#define BIT_SPI_RXCTL_RWCEN (1U << 3U) // HRM 15-66 (RWCEN, bit 3)
#define POS_SPI_RXCTL_RDR   4U         // HRM 15-65 (RDR,   bits 4..6)

// SPI_TXCTL bits (Table 15-28, HRM 15-78).
#define BIT_SPI_TXCTL_TEN   (1U << 0U) // HRM 15-80 (TEN,   bit 0)
#define BIT_SPI_TXCTL_TTI   (1U << 2U) // HRM 15-79 (TTI,   bit 2)
#define BIT_SPI_TXCTL_TWCEN (1U << 3U) // HRM 15-79 (TWCEN, bit 3)
#define POS_SPI_TXCTL_TDR   4U         // HRM 15-79 (TDR,   bits 4..6)
#define BIT_SPI_TXCTL_TDU   (1U << 8U) // HRM 15-79 (TDU,   bit 8)

// SPI_STAT bits (Table 15-25, HRM 15-70).
#define BIT_SPI_STAT_SPIF (1U << 0U)  // HRM 15-74 (SPIF, bit 0)
#define BIT_SPI_STAT_ROR  (1U << 4U)  // HRM 15-73 (ROR,  bit 4)
#define BIT_SPI_STAT_TUR  (1U << 5U)  // HRM 15-73 (TUR,  bit 5)
#define BIT_SPI_STAT_TC   (1U << 6U)  // HRM 15-73 (TC,   bit 6)
#define BIT_SPI_STAT_MF   (1U << 7U)  // HRM 15-73 (MF,   bit 7)
#define BIT_SPI_STAT_RS   (1U << 8U)  // HRM 15-72 (RS,   bit 8)
#define BIT_SPI_STAT_TS   (1U << 9U)  // HRM 15-72 (TS,   bit 9)
#define BIT_SPI_STAT_RF   (1U << 10U) // HRM 15-72 (RF,   bit 10)
#define BIT_SPI_STAT_TF   (1U << 11U) // HRM 15-72 (TF,   bit 11)
#define POS_SPI_STAT_RFS  12U         // HRM 15-71 (RFS,  bits 12..14)
#define POS_SPI_STAT_TFS  16U         // HRM 15-71 (TFS,  bits 16..18)
#define BIT_SPI_STAT_RFE  (1U << 22U) // HRM 15-71 (RFE,  bit 22)
#define BIT_SPI_STAT_TFF  (1U << 23U) // HRM 15-71 (TFF,  bit 23)

// SPI_SLVSEL bits (Table 15-24, HRM 15-67).
#define BIT_SPI_SLVSEL_SSE1  (1U << 1U) // HRM 15-69 (SSE1,  bit 1)
#define BIT_SPI_SLVSEL_SSEL1 (1U << 9U) // HRM 15-68 (SSEL1, bit 9)

// Slave-mode SLVSEL value: SSE1..SSE7 all disabled (low byte = 0x00,
// so the peripheral does not drive any SEL output while acting as a
// slave) and SSEL1..SSEL7 all high (bits 9..15 = 0xFE00) to mirror
// CCES adi_spi_2156x.c :: DEFAULT_SPISLVSEL_OUTPUT. The SSEL high
// bits only matter when SSE is set, but CCES writes them regardless
// so that a subsequent master reconfigure starts from the expected
// "all chips deselected" state.
#define SPI_SLVSEL_SLAVE_IDLE 0x0000FE00U

// PORTB registers for SPI0 alternate-function pins.
#define REG_PORTB_FER 0x31004080U // HRM 12-41
#define REG_PORTB_MUX 0x310040B0U // HRM 12-61

#endif // REGS_H
