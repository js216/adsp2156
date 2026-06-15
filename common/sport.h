// SPDX-License-Identifier: MIT
// sport.h --- SHARC+ Serial Port (SPORT) core-polled driver
// Copyright (c) 2026 Jakob Kastelic

// Each SPORT top module on the 21569 is two independent half
// SPORTs (A and B) that share a register block. This driver
// configures one half at a time for plain DSP-serial mode and
// provides core-polled primary-channel read/write helpers with
// a timeout. I2S, left-justified, and TDM multichannel need
// their own init paths and are not covered here.
//
// The driver does not own SRU / DAI routing: pin buffers,
// cross-connects, and clock sources are the caller's problem.

#ifndef SPORT_H
#define SPORT_H

#include <stdbool.h>
#include <stdint.h>

// Which of the eight SPORT top modules.
enum sport_id {
   SPORT_ID_0 = 0,
   SPORT_ID_1,
   SPORT_ID_2,
   SPORT_ID_3,
   SPORT_ID_4,
   SPORT_ID_5,
   SPORT_ID_6,
   SPORT_ID_7,
   SPORT_ID_COUNT
};

// Which half within a SPORT top module.
enum sport_half {
   SPORT_HALF_A = 0,
   SPORT_HALF_B = 1,
};

// Parameter block for sport_dsp_serial_init.
struct sport_dsp_cfg {
   uint32_t word_bits; // 4..32 data bits per word
   uint32_t clkdiv;    // bit clock = SCLK0 / (clkdiv + 1)
   uint32_t fsdiv;     // FS period  = (fsdiv + 1) bit clocks
   bool is_tx;         // true for transmit half, false for receive
   bool internal_clk;  // true = master (drive bit clock)
   bool internal_fs;   // true = master (drive frame sync)
   bool late_fs;       // LAFS: FS aligned with first data bit
   bool data_indep_fs; // DIFS: generate FS regardless of buffer state
   bool sample_rising; // CKRE: sample input on rising clock edge
};

// Configure one half of a SPORT for DSP-serial mode.
//   id:   which SPORT module (SPORT_ID_0..7).
//   half: SPORT_HALF_A or SPORT_HALF_B.
//   cfg:  pointer to the serial-mode parameter block.
// Clears all control registers, writes CTL + DIV per cfg, and
// leaves the half disabled (SPENPRI = 0).
void sport_dsp_serial_init(const enum sport_id id, const enum sport_half half,
                           const struct sport_dsp_cfg *cfg);

// Set SPENPRI (primary channel enable bit).
//   id:   which SPORT module.
//   half: which half.
void sport_enable(const enum sport_id id, const enum sport_half half);

// Clear SPENPRI.
//   id:   which SPORT module.
//   half: which half.
void sport_disable(const enum sport_id id, const enum sport_half half);

// Polled 32-bit read from the primary RX buffer.
//   id:       which SPORT module.
//   half:     which half.
//   word_out: pointer receiving the 32-bit value (written only
//             on success).
//   timeout:  maximum status-poll iterations.
//   returns:  0 on success, -1 on timeout.
int sport_read(const enum sport_id id, const enum sport_half half,
               uint32_t *word_out, const uint32_t timeout);

// Unchecked raw write. No DXSPRI poll.
//   id:   which SPORT module.
//   half: which half.
//   word: 32-bit value to push into the TX buffer.
void sport_write_raw(const enum sport_id id, const enum sport_half half,
                     const uint32_t word);

// Unchecked raw read. No DXSPRI poll.
//   id:      which SPORT module.
//   half:    which half.
//   returns: the 32-bit value popped from the RX buffer.
uint32_t sport_read_raw(const enum sport_id id, const enum sport_half half);

// Return true when the primary RX buffer is not empty.
//   id:      which SPORT module.
//   half:    which half.
//   returns: true if a word can be read.
bool sport_rx_ready(const enum sport_id id, const enum sport_half half);

// True when the primary TX buffer is not full and can accept a word.
bool sport_tx_ready(const enum sport_id id, const enum sport_half half);

// Install a purely-internal loopback route that feeds half A's
// clock, frame sync and primary data output back into half B's
// corresponding inputs for the given SPORT. No DAI pins leave
// the chip for SPORT0..2 and SPORT4..6 (the signals stay in
// the SRU crossbar). SPORT3 and SPORT7 need a pin-buffer
// pass-through for the clock and frame sync because neither
// SPT{3,7}_ACLK_O nor SPT{3,7}_AFS_O is a group-A source
// signal; the helper drives DAI pin buffers 11 and 12 as a
// side effect, but nothing on the test rig is connected to
// those pads.
//
//   id: which of the eight SPORT top modules to loop.
//
// The caller still has to configure both halves with
// sport_dsp_serial_init() and turn them on with sport_enable()
// after this call returns.
void sport_install_internal_loopback(const enum sport_id id);

// Clear sticky error bits (DERRPSTAT / DERRSSTAT / FSERRSTAT)
// in the SPORT_ERR register (W1C, bits 4..6).
//   id:   which SPORT module.
//   half: which half.
void sport_clear_errors(const enum sport_id id, const enum sport_half half);

// Check primary-data error flag (DERRPRI, CTL bit 29).
//   id:      which SPORT module.
//   half:    which half.
//   returns: true if TX underflow or RX overflow has occurred.
bool sport_has_error(const enum sport_id id, const enum sport_half half);

// Force the DAI pin-buffer output enables for the three default
// half-A pins of the given SPORT (AD0, ACLK, AFS) to static HIGH
// and enable the DAI pad input buffers. For SPORT4, also explicitly
// routes SPT4_AD0_O, SPT4_ACLK_O and SPT4_AFS_O onto DAI1
// PB01/PB02/PB04. This is needed for external loopback through
// off-chip jumpers because the SPORT's own PBEN_O signals do not
// automatically assert in plain DSP-serial mode.
//
//   id: which SPORT module to enable. The default pin-to-
//       SPORT mapping is used (e.g. SPORT4 = PB01/PB02/PB04
//       on DAI1).
void sport_enable_external_pins(const enum sport_id id);

// Route SPORT0 half-A TX signals onto the wired DAI0 PB05/PB07/PB08
// pins instead of the default SPORT0A pin set. The live FPGA-DSP scan
// maps those pins to R5/T5/R9; SPORT0A's default AD0 pin is not wired
// on this bench. This helper only changes SRU/PBEN routing. The caller
// still configures SPORT0A as a transmitter and enables it normally.
void sport_route_sport0a_to_wired_pins(void);

// Route SPORT half-A TX signals onto an arbitrary wired DAI pin triple.
// `dai_idx` is 0 or 1. Pin numbers are DAI PB numbers (1..12, 19, 20).
// The source SPORT must belong to the selected DAI half (SPORT0..3 for
// DAI0, SPORT4..7 for DAI1).
void sport_route_clk_copy(const enum sport_id id, uint32_t dai_idx,
                          uint32_t pin);
void sport_route_tx_to_pins(const enum sport_id id, uint32_t dai_idx,
                            uint32_t ad0_pin, uint32_t aclk_pin,
                            uint32_t afs_pin);

// Route arbitrary wired DAI input pins into SPORT half-B. The FPGA drives
// the selected pin triple; this helper forces those pins to inputs and
// routes PBxx_O into the SPORT B clock/frame/data destinations.
void sport_route_rx_from_pins(const enum sport_id id, uint32_t dai_idx,
                              uint32_t ad0_pin, uint32_t aclk_pin,
                              uint32_t afs_pin);

// Route a SPORT half-B receiver that is also the SPORT clock/frame-sync
// master. The data pin is an input into BD0; the ACLK/BFS pins are driven
// from the half-B clock and frame-sync outputs.
void sport_route_rx_master_to_pins(const enum sport_id id, uint32_t dai_idx,
                                   uint32_t ad0_pin, uint32_t bclk_pin,
                                   uint32_t bfs_pin);

// Same as sport_route_rx_master_to_pins(), but for half-A: AD0 is routed
// from the DAI pin into the SPORT receiver, while ACLK/AFS are driven from
// the half-A clock/frame outputs.
void sport_route_rx_master_a_to_pins(const enum sport_id id, uint32_t dai_idx,
                                     uint32_t ad0_pin, uint32_t aclk_pin,
                                     uint32_t afs_pin);

#endif // SPORT_H
