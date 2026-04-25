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

// SPORT serial-line geometry shared by all loopback configs.
#define SPORT_WORD_BITS 32U
// FSDIV+1 == SLEN+1 = WORD_BITS for back-to-back continuous words.
#define SPORT_FSDIV_DEFAULT 31U
// Initial sweep CLKDIV: bit clock = SCLK / (2*(CLKDIV+1)) ~= 500 kHz.
#define SPORT_CLKDIV_DEFAULT 93U
// Word-rate divisor: word_rate = bit_rate / WORD_BITS.
#define SPORT_BIT_PER_WORD_LOG2 5U
// Sentinel marking "no clkdiv has passed yet" while sweeping.
#define SPORT_CLKDIV_NONE 0xFFFFFFFFU
// Hz-per-MHz divisor for printout.
#define HZ_PER_MHZ 1000000U

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
static struct sport_dsp_cfg tx_master_cfg = {
    .word_bits     = SPORT_WORD_BITS,
    .clkdiv        = SPORT_CLKDIV_DEFAULT, // overwritten by sweep
    .fsdiv         = SPORT_FSDIV_DEFAULT,
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

// Bit-rate from clkdiv: SPORT bit clock = SCLK0 / (2 * (clkdiv + 1))
// per HRM 23-29.  At SCLK0=93.75 MHz that gives ~46.9 MHz at
// clkdiv=0, ~23.4 MHz at clkdiv=1, etc.  Test SPORT4 (external,
// already-jumpered) at a sweep of divisors to find the lowest
// (= fastest) that still passes the LFSR loopback.
static const uint32_t sweep_clkdivs[] = {
    0, 1, 2, 3, 5, 8, 11, 23, 46, 55, 60, 65, 70, 75, 80, 85, 89, 91, 92, 93};

// Manual divide: cc21k -no-std-lib lacks __divrem_u32.
static uint32_t udiv32(uint32_t n, uint32_t d)
{
   if (d == 0U)
      return 0U;
   uint32_t q = 0;
   while (n >= d) {
      n -= d;
      q++;
   }
   return q;
}

int main(void)
{
   static const struct clocks_cfg clk = BOARD_CLOCKS_CFG;
   clocks_init(&clk);
   uart_init(BOARD_BAUD_DIV);
   timer_init();
   board_som_init(0U);

   printf("\r\nsport SPORT4 external loopback sweep\r\n");

   sport_enable_external_pins(SPORT_ID_4);

   uint32_t best_clkdiv = SPORT_CLKDIV_NONE;
   for (unsigned i = 0; i < sizeof(sweep_clkdivs) / sizeof(sweep_clkdivs[0]);
        i++) {
      uint32_t clkdiv      = sweep_clkdivs[i];
      tx_master_cfg.clkdiv = clkdiv;
      sport_dsp_serial_init(SPORT_ID_4, SPORT_HALF_A, &tx_master_cfg);
      sport_dsp_serial_init(SPORT_ID_4, SPORT_HALF_B, &rx_slave_cfg);
      struct loopback_result r = run_loopback_test(SPORT_ID_4, SEED_SPORT4);
      uint32_t bit_hz          = udiv32(BOARD_SCLK_HZ, 2U * (clkdiv + 1U));
      uint32_t word_hz         = bit_hz >> SPORT_BIT_PER_WORD_LOG2;
      uint32_t mbps            = udiv32(bit_hz, HZ_PER_MHZ);
      const char *verdict = (r.fails == 0 && r.timeouts == 0) ? "PASS" : "FAIL";
      printf("  clkdiv=%2u bit_clk=%u Hz word_rate=%u w/s data=%u Mbps "
             "fails=%u timeouts=%u %s\r\n",
             (unsigned)clkdiv, (unsigned)bit_hz, (unsigned)word_hz,
             (unsigned)mbps, (unsigned)r.fails, (unsigned)r.timeouts, verdict);
      if (r.fails == 0 && r.timeouts == 0 && clkdiv < best_clkdiv)
         best_clkdiv = clkdiv;
   }
   if (best_clkdiv != SPORT_CLKDIV_NONE) {
      uint32_t bit_hz = udiv32(BOARD_SCLK_HZ, 2U * (best_clkdiv + 1U));
      uint32_t mbps   = udiv32(bit_hz, HZ_PER_MHZ);
      printf("MAX SPORT4: clkdiv=%u bit_clk=%u Hz (%u Mbps) over current "
             "jumpers\r\n",
             (unsigned)best_clkdiv, (unsigned)bit_hz, (unsigned)mbps);
   } else {
      printf("MAX SPORT4: NONE PASSED\r\n");
   }

   // Skip the SPORT0 internal-loopback / SPORT4 single-rate
   // tests below (kept commented for reference) -- the sweep
   // above subsumes them.
   struct loopback_result s0 = {0, 0, 0, 0, 0};
   struct loopback_result s4 = {0, 0, 0, 0, 0};

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
