// SPDX-License-Identifier: MIT
// main.c --- SPORT loopback datapath tests
// Copyright (c) 2026 Jakob Kastelic

// Two independent SPORT pair tests run back-to-back against
// the common/sport driver:
//
//   1. SPORT0 internal loopback (no wires). SPORT0A drives
//      clock, frame sync and data; all three are routed
//      internally back to SPORT0B through the DAI0 SRU
//      crossbar (groups A, B, C). No DAI pins involved,
//      nothing leaves the chip.
//
//   2. SPORT4 external loopback (three jumpers on P13 of the
//      EV-SOMCRR-EZLITE carrier). SPORT4A drives clock, frame
//      sync and data out onto DAI1 pin buffers 01/03/04 using
//      the DAI1 default SRU routing; SPORT4B reads them back
//      from DAI1 pin buffers 05/07/08. The required jumpers:
//
//          P13 pin 2  <-> P13 pin 10   (serial data)
//          P13 pin 6  <-> P13 pin 14   (serial clock)
//          P13 pin 8  <-> P13 pin 16   (frame sync)
//
// Each test pushes a 32-bit LFSR sequence into the TX half and
// re-derives the expected stream on the RX half for comparison.
// A mismatch prints the index and the differing words; an RX
// timeout indicates a wiring problem on the external side.

#include "board.h"
#include "clocks.h"
#include "sport.h"
#include "timer.h"
#include "uart.h"
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

// Number of words per test. At a ~1 MHz bit clock 32-bit words
// take 32 us each; 4096 words is ~131 ms. Two tests back-to-back
// fit comfortably inside the five-second hardware window.
#define N_WORDS 4096U

// Polled-read / polled-write iteration cap. Far bigger than any
// transit should need so a miswire prints a timeout instead of
// hanging the core forever.
#define POLL_TIMEOUT 200000U

// 32-bit maximum-length LFSR feedback polynomial taps.
// Polynomial: x^32 + x^22 + x^2 + x^1, period 2^32 - 1.
#define LFSR_TAP_31 31U
#define LFSR_TAP_21 21U

// Test seeds: different per SPORT to catch stray cross-connects.
#define SEED_SPORT0 0xACE1BEEFU
#define SEED_SPORT4 0x12345678U

// Number of times to repeat the result block so the test rig's
// UART capture ring always has at least one full copy.
#define RESULT_REPEATS  6U
#define RESULT_DELAY_MS 300U

// 32-bit maximum-length LFSR (taps = 32, 22, 2, 1). Any
// non-zero seed is fine.
static uint32_t lfsr_next(uint32_t *state)
{
   uint32_t s = *state;
   uint32_t bit =
       ((s >> LFSR_TAP_31) ^ (s >> LFSR_TAP_21) ^ (s >> 1U) ^ s) & 1U;
   s      = (s << 1U) | bit;
   *state = s;
   return s;
}

// DSP-serial config shared by both tests. Half A uses it as a
// master transmitter (own clock, own frame sync, data-
// independent FS); half B uses it as a slave receiver (external
// clock and frame sync). LAFS = 1 so the FS pulse aligns with
// the first data bit instead of the cycle before it -- required
// when FSDIV+1 == SLEN+1 for back-to-back continuous words.
static const struct sport_dsp_cfg tx_master_cfg = {
    .word_bits     = 32,
    .clkdiv        = 93, // ~1 MHz bit clock from 93.75 MHz SCLK0
    .fsdiv         = 31, // one FS per 32-bit word
    .is_tx         = true,
    .internal_clk  = true,
    .internal_fs   = true,
    .late_fs       = true,
    .data_indep_fs = true,
    .sample_rising = true,
};

static const struct sport_dsp_cfg rx_slave_cfg = {
    .word_bits     = 32,
    .clkdiv        = 0, // unused (external clock)
    .fsdiv         = 0, // unused (external FS)
    .is_tx         = false,
    .internal_clk  = false,
    .internal_fs   = false,
    .late_fs       = true,
    .data_indep_fs = false,
    .sample_rising = true,
};

// Loopback test result.
struct loopback_result {
   uint32_t fails;
   uint32_t first_idx;
   uint32_t first_got;
   uint32_t first_exp;
   uint32_t timeouts;
};

// One push/pop/compare pass against a SPORT pair. Assumes the
// caller has already installed the required SRU / pad routing
// and configured the two halves. Enables A and B (B first so
// the receiver is armed before the first clock), runs N_WORDS
// words, disables both halves, and returns the result.
static struct loopback_result run_loopback_test(enum sport_id id, uint32_t seed)
{
   struct loopback_result res = {0, 0, 0, 0, 0};
   uint32_t tx_state          = seed;
   uint32_t rx_state          = seed;

   // Preload two words so the hardware has something to
   // transmit on the first two frame syncs after enable. The
   // primary TX buffer is two words deep (HRM 23-40).
   sport_write_raw(id, SPORT_HALF_A, lfsr_next(&tx_state));
   sport_write_raw(id, SPORT_HALF_A, lfsr_next(&tx_state));

   sport_enable(id, SPORT_HALF_B);
   sport_enable(id, SPORT_HALF_A);

   for (uint32_t i = 0; i < N_WORDS; i++) {
      uint32_t got = 0;
      if (sport_read(id, SPORT_HALF_B, &got, POLL_TIMEOUT) < 0) {
         res.timeouts++;
         break;
      }

      uint32_t exp = lfsr_next(&rx_state);
      sport_write_raw(id, SPORT_HALF_A, lfsr_next(&tx_state));

      if (got != exp) {
         if (res.fails == 0) {
            res.first_idx = i;
            res.first_got = got;
            res.first_exp = exp;
         }
         res.fails++;
      }
   }

   sport_disable(id, SPORT_HALF_A);
   sport_disable(id, SPORT_HALF_B);

   return res;
}

int main(void)
{
   static const struct clocks_cfg clk = BOARD_CLOCKS_CFG;
   clocks_init(&clk);
   uart_init(BOARD_BAUD_DIV);
   timer_init();

   // Test 1: SPORT0 internal loopback.
   sport_install_internal_loopback(SPORT_ID_0);
   sport_dsp_serial_init(SPORT_ID_0, SPORT_HALF_A, &tx_master_cfg);
   sport_dsp_serial_init(SPORT_ID_0, SPORT_HALF_B, &rx_slave_cfg);

   struct loopback_result s0 = run_loopback_test(SPORT_ID_0, SEED_SPORT0);

   // Test 2: SPORT4 external loopback via P13 jumpers. A
   // different LFSR seed catches any stray cross-connect
   // between the two tests.
   sport_enable_external_pins(SPORT_ID_4);
   sport_dsp_serial_init(SPORT_ID_4, SPORT_HALF_A, &tx_master_cfg);
   sport_dsp_serial_init(SPORT_ID_4, SPORT_HALF_B, &rx_slave_cfg);

   struct loopback_result s4 = run_loopback_test(SPORT_ID_4, SEED_SPORT4);

   const char *s0_verdict =
       (s0.fails == 0 && s0.timeouts == 0) ? "PASS" : "FAIL";
   const char *s4_verdict =
       (s4.fails == 0 && s4.timeouts == 0) ? "PASS" : "FAIL";

   // Repeat the result block so the rig's UART capture ring
   // always has at least one full copy.
   for (uint32_t r = 0; r < RESULT_REPEATS; r++) {
      printf("sport0 internal %s f=%x/%x t=%x @%x g=%x e=%x\r\n", s0_verdict,
             s0.fails, N_WORDS, s0.timeouts, s0.first_idx, s0.first_got,
             s0.first_exp);
      printf("sport4 external %s f=%x/%x t=%x @%x g=%x e=%x\r\n", s4_verdict,
             s4.fails, N_WORDS, s4.timeouts, s4.first_idx, s4.first_got,
             s4.first_exp);
      delay_ms(RESULT_DELAY_MS);
   }

   for (;;) {
   }
}
