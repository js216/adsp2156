// SPDX-License-Identifier: MIT
// main.c --- SPORT4 half-B long PRBS DMA receiver
// Copyright (c) 2026 Jakob Kastelic

#include "board.h"
#include "clocks.h"
#include "dma.h"
#include "gpio.h"
#include "regs.h"
#include "sport.h"
#include "timer.h"
#include "uart.h"
#include <stdbool.h>
#include <stdint.h>

#ifndef TOTAL_BYTES
#define TOTAL_BYTES 67108864ULL
#endif
#ifndef TOTAL_WORDS
#define TOTAL_WORDS 16777216U
#endif
#ifndef HALF_WORDS
#define HALF_WORDS        32768U
#endif
#ifndef RX_SAMPLE_RISING
#define RX_SAMPLE_RISING  1
#endif
#ifndef RX_LATE_FS
#define RX_LATE_FS        0
#endif
#ifndef RX_DATA_INDEP_FS
#define RX_DATA_INDEP_FS  0
#endif
#ifndef RX_SHIFT_LEFT_1
#define RX_SHIFT_LEFT_1   0
#endif
#ifndef SPORT_SCLK_HZ
#define SPORT_SCLK_HZ     59375000U
#endif
#ifndef SPORT_CLKDIV
#define SPORT_CLKDIV      2U
#endif
#define BIT_CLK_HZ        (SPORT_SCLK_HZ / (SPORT_CLKDIV + 1U))
#ifndef SPORT_FSDIV
#define SPORT_FSDIV       31U
#endif
#define CAPTURE_WORDS     (TOTAL_WORDS + (RX_SHIFT_LEFT_1 ? 1U : 0U))
#define MAX_RATE_BPS      32000000U
#define MIN_RATE_BPS      30000000U
#define START_TIMEOUT     1000U
#define PRBS31_SEED       0x7FFFFFFFU
#define SCLK_HZ_64        ((uint64_t)SPORT_SCLK_HZ)
#define BITS_PER_BYTE_64  8ULL
#define SPORT4B_ERR       0x310024A0U
#define LOCAL_BAUD_DIV    ((SPORT_SCLK_HZ + (BOARD_BAUD / 2U)) / BOARD_BAUD)

#pragma section("seg_l2_bss", NO_INIT)
static uint32_t rx_ring[2][HALF_WORDS];
#pragma section("seg_l2_bss", NO_INIT)
static struct dma_dscl rx_desc[2];

static uint32_t errors = 0U;
static uint32_t timeouts = 0U;
static uint32_t overruns = 0U;
static uint32_t wrap_misses = 0U;
static uint32_t got_words = 0U;
static uint32_t prbs_state = PRBS31_SEED;
static int32_t firsterr = -1;
static uint32_t firstgot = 0U;
static uint32_t firstexp = 0U;
static uint64_t elapsed_ticks64 = 0ULL;
static bool locked = false;
static uint8_t prbs8_table[2048];
static uint32_t dbg_got[16];
static uint32_t dbg_exp[16];
static uint32_t err_idx[16];
static uint32_t err_got[16];
static uint32_t err_exp[16];
static uint32_t err_saved = 0U;
static uint32_t raw_words = 0U;
static uint32_t align_prev = 0U;
static bool align_prev_valid = false;

static void put_str(const char *s)
{
   while (*s != '\0')
      uart_putc(*s++);
}

static void put_u32(uint32_t v)
{
   static const uint32_t pow10[10] = {
       1000000000U, 100000000U, 10000000U, 1000000U, 100000U,
       10000U,      1000U,      100U,      10U,      1U,
   };
   if (v == 0U) {
      uart_putc('0');
      return;
   }
   unsigned started = 0U;
   for (unsigned i = 0U; i < 10U; i++) {
      uint32_t d = pow10[i];
      char c = '0';
      while (v >= d) {
         v -= d;
         c = (char)(c + 1);
      }
      if (c != '0')
         started = 1U;
      if (started != 0U)
         uart_putc(c);
   }
}

static void put_i32(int32_t v)
{
   if (v < 0) {
      uart_putc('-');
      put_u32((uint32_t)(-v));
   } else {
      put_u32((uint32_t)v);
   }
}

static uint64_t udiv64(uint64_t n, uint64_t d)
{
   uint64_t q = 0ULL;
   uint64_t r = 0ULL;
   for (int bit = 63; bit >= 0; bit--) {
      r = (r << 1) | ((n >> (unsigned)bit) & 1ULL);
      if (r >= d) {
         r -= d;
         q |= (1ULL << (unsigned)bit);
      }
   }
   return q;
}

static void put_u64(uint64_t v)
{
   static const uint64_t pow10[20] = {
       10000000000000000000ULL,
       1000000000000000000ULL,
       100000000000000000ULL,
       10000000000000000ULL,
       1000000000000000ULL,
       100000000000000ULL,
       10000000000000ULL,
       1000000000000ULL,
       100000000000ULL,
       10000000000ULL,
       1000000000ULL,
       100000000ULL,
       10000000ULL,
       1000000ULL,
       100000ULL,
       10000ULL,
       1000ULL,
       100ULL,
       10ULL,
       1ULL,
   };
   if (v == 0ULL) {
      uart_putc('0');
      return;
   }
   unsigned started = 0U;
   for (unsigned i = 0U; i < 20U; i++) {
      uint64_t d = pow10[i];
      char c = '0';
      while (v >= d) {
         v -= d;
         c = (char)(c + 1);
      }
      if (c != '0')
         started = 1U;
      if (started != 0U)
         uart_putc(c);
   }
}

static void put_hex32(uint32_t v)
{
   static const char hex[] = "0123456789abcdef";
   for (int i = 7; i >= 0; i--)
      uart_putc(hex[(v >> ((unsigned)i * 4U)) & 0xFU]);
}

static void prbs8_init(void)
{
   for (uint32_t idx = 0U; idx < 2048U; idx++) {
      uint32_t s = idx << 20U;
      uint32_t b = 0U;
      for (uint32_t i = 0U; i < 8U; i++) {
         uint32_t new_bit = ((s >> 30U) ^ (s >> 27U)) & 1U;
         s = ((s << 1U) | new_bit) & 0x7FFFFFFFU;
         b = (b << 1U) | new_bit;
      }
      prbs8_table[idx] = (uint8_t)b;
   }
}

// PRBS-31, polynomial x^31 + x^28 + 1, seed 0x7fffffff.
// Generates serial bits with new_bit = state[30] ^ state[27] and packs
// 32 output bits per SPORT word, first generated bit into word bit 31.
static inline uint32_t prbs31_word(uint32_t *state)
{
   uint32_t s = *state;
   uint32_t b0 = prbs8_table[s >> 20U];
   s = ((s << 8U) & 0x7FFFFFFFU) | b0;
   uint32_t b1 = prbs8_table[s >> 20U];
   s = ((s << 8U) & 0x7FFFFFFFU) | b1;
   uint32_t b2 = prbs8_table[s >> 20U];
   s = ((s << 8U) & 0x7FFFFFFFU) | b2;
   uint32_t b3 = prbs8_table[s >> 20U];
   s = ((s << 8U) & 0x7FFFFFFFU) | b3;
   *state = s;
   uint32_t word = (b0 << 24U) | (b1 << 16U) | (b2 << 8U) | b3;
   return word;
}

static inline void accum_elapsed_ticks(uint64_t *elapsed_ticks64,
                                       uint32_t *last_tick)
{
   uint32_t now = timer_ticks();
   *elapsed_ticks64 += (uint64_t)(uint32_t)(now - *last_tick);
   *last_tick = now;
}

static uint32_t dma_ring_half(void)
{
   uint32_t addr = dma_addr_cur(DMA_CH_SPORT4_B);
   uint32_t b0 = (uint32_t)&rx_ring[0][0];
   uint32_t b1 = (uint32_t)&rx_ring[1][0];
   uint32_t bytes = HALF_WORDS * 4U;

   if (addr >= b1 && addr < (b1 + bytes))
      return 1U;
   if (addr >= b0 && addr < (b0 + bytes))
      return 0U;
   return 2U;
}

static bool dma_still_on_half(uint32_t completed_half)
{
   for (uint32_t i = 0U; i < 10000U; i++) {
      if (dma_ring_half() != completed_half)
         return false;
   }
   return dma_ring_half() == completed_half;
}

static void check_word(uint32_t word, bool *locked, uint32_t *prbs_state,
                       uint32_t *got_words, uint32_t *errors,
                       int32_t *firsterr, uint32_t *firstgot,
                       uint32_t *firstexp)
{
   uint32_t expected = prbs31_word(prbs_state);
   if (*got_words < 16U) {
      dbg_got[*got_words] = word;
      dbg_exp[*got_words] = expected;
   }
   if (word != expected) {
      if (err_saved < 16U) {
         err_idx[err_saved] = *got_words;
         err_got[err_saved] = word;
         err_exp[err_saved] = expected;
         err_saved++;
      }
      if (*firsterr < 0) {
         *firsterr = (int32_t)*got_words;
         *firstgot = word;
         *firstexp = expected;
      }
      if (*errors != UINT32_MAX)
         (*errors)++;
   }
   (*got_words)++;
   *locked = (*got_words != 0U);
}

static void process_words(const uint32_t *buf, uint32_t nwords, bool *locked,
                          uint32_t *prbs_state, uint32_t *got_words,
                          uint32_t *errors, int32_t *firsterr,
                          uint32_t *firstgot, uint32_t *firstexp)
{
   for (uint32_t i = 0U; i < nwords && raw_words < CAPTURE_WORDS; i++) {
#if RX_SHIFT_LEFT_1
      if (align_prev_valid && *got_words < TOTAL_WORDS) {
         uint32_t aligned = (align_prev << 1) | (buf[i] >> 31);
         check_word(aligned, locked, prbs_state, got_words, errors, firsterr,
                    firstgot, firstexp);
      }
      align_prev = buf[i];
      align_prev_valid = true;
#else
      if (*got_words < TOTAL_WORDS)
         check_word(buf[i], locked, prbs_state, got_words, errors, firsterr,
                    firstgot, firstexp);
#endif
      raw_words++;
   }
}

static struct sport_dsp_cfg rx_slave_cfg = {
    .word_bits     = 32,
    .clkdiv        = SPORT_CLKDIV,
    .fsdiv         = SPORT_FSDIV,
    .is_tx         = false,
    .internal_clk  = true,
    .internal_fs   = true,
    .late_fs       = (RX_LATE_FS != 0),
    .data_indep_fs = (RX_DATA_INDEP_FS != 0),
    .sample_rising = (RX_SAMPLE_RISING != 0),
};

int main(void)
{
   static const struct clocks_cfg clk =
#if SPORT_SCLK_HZ == 60000000U
      CLOCKS_CFG_SCLK0_60MHZ;
#elif SPORT_SCLK_HZ == 59375000U
      CLOCKS_CFG_SCLK0_59MHZ;
#else
      BOARD_CLOCKS_CFG;
#endif
   clocks_init(&clk);
   uart_init(LOCAL_BAUD_DIV);
   timer_init();
   board_som_init(0U);
   gpio_make_output(GPIO_DAI1_06);
   gpio_write(GPIO_DAI1_06, false);
   prbs8_init();

   put_str("\r\nsport_fpga_tx_prbs_long boot prbs31 poly=x^31+x^28+1 seed=0x7FFFFFFF pack=msb_first_output_bits\r\n");

   put_str("sport_route_start\r\n");
   sport_route_rx_master_to_pins(SPORT_ID_4, 1U, 5U, 7U, 8U);
   put_str("sport_init_start\r\n");
   sport_dsp_serial_init(SPORT_ID_4, SPORT_HALF_B, &rx_slave_cfg);
   put_str("sport_clear_start\r\n");
   sport_clear_errors(SPORT_ID_4, SPORT_HALF_B);
   put_str("sport_setup_done\r\n");

#ifdef DIAG_FIRST_WORDS
   gpio_write(GPIO_DAI1_06, true);
   delay_us(10U);
   sport_enable(SPORT_ID_4, SPORT_HALF_B);
   put_str("sport_diag start\r\n");
   uint32_t nread = 0U;
   uint32_t deadline_diag = timer_ticks() + (SPORT_SCLK_HZ * 4U);
   while ((int32_t)(timer_ticks() - deadline_diag) < 0 && nread < 16U) {
      uint32_t word = 0U;
      if (sport_read(SPORT_ID_4, SPORT_HALF_B, &word, START_TIMEOUT) == 0) {
         put_str("sport_diag word");
         put_u32(nread);
         put_str("=0x");
         put_hex32(word);
         put_str("\r\n");
         nread++;
      }
   }
   put_str("sport_diag nread=");
   put_u32(nread);
   put_str(" err=0x");
   put_hex32(MMR(SPORT4B_ERR));
   put_str("\r\n");
   for (;;) {
   }
#endif

   put_str("normal_path_start\r\n");
   put_str("dma_pp_cfg\r\n");
   dma_pingpong_rx_config(DMA_CH_SPORT4_B, &rx_ring[0][0], &rx_ring[1][0],
                          HALF_WORDS, rx_desc);
   put_str("dma_pp_armed\r\n");
   gpio_write(GPIO_DAI1_06, true);
   put_str("run_asserted\r\n");
   delay_us(10U);
   sport_enable(SPORT_ID_4, SPORT_HALF_B);
   put_str("sport_rx_enabled\r\n");
   locked = true;

   uint32_t last_tick = timer_ticks();
   uint32_t halves = 0U;
   while (raw_words < CAPTURE_WORDS) {
      uint32_t deadline = last_tick + (SPORT_SCLK_HZ * 3U);
      while (!dma_wrap_check(DMA_CH_SPORT4_B)) {
         accum_elapsed_ticks(&elapsed_ticks64, &last_tick);
         if ((int32_t)(last_tick - deadline) >= 0) {
            timeouts++;
            break;
         }
      }
      if (timeouts != 0U)
         break;

      uint32_t completed = halves & 1U;
      bool final_half = (raw_words + HALF_WORDS) >= CAPTURE_WORDS;
      if (final_half) {
         dma_disable(DMA_CH_SPORT4_B);
         dma_wait_idle(DMA_CH_SPORT4_B);
      }
      if (dma_still_on_half(completed)) {
         overruns++;
         break;
      }
      process_words(&rx_ring[completed][0], HALF_WORDS, &locked, &prbs_state,
                    &got_words, &errors, &firsterr, &firstgot, &firstexp);
      if (!final_half)
         accum_elapsed_ticks(&elapsed_ticks64, &last_tick);
      if (sport_has_error(SPORT_ID_4, SPORT_HALF_B)) {
         overruns++;
         break;
      }
      halves++;
      if (halves > ((CAPTURE_WORDS / HALF_WORDS) + 2048U)) {
         timeouts++;
         break;
      }
   }

   dma_disable(DMA_CH_SPORT4_B);
   dma_wait_idle(DMA_CH_SPORT4_B);

   sport_disable(SPORT_ID_4, SPORT_HALF_B);
   bool sport_error = sport_has_error(SPORT_ID_4, SPORT_HALF_B);
   uint32_t sport_err = MMR(SPORT4B_ERR);
   uint64_t bytes = (uint64_t)got_words * 4ULL;
   uint64_t rate_bps = 0ULL;
   if (elapsed_ticks64 != 0ULL) {
      rate_bps = udiv64((uint64_t)bytes * BITS_PER_BYTE_64 * SCLK_HZ_64,
                        elapsed_ticks64);
   }

   bool pass = (bytes == TOTAL_BYTES && got_words == TOTAL_WORDS &&
                errors == 0U && firsterr == -1 && timeouts == 0U &&
                overruns == 0U && wrap_misses == 0U && locked && !sport_error &&
                BIT_CLK_HZ >= MIN_RATE_BPS && BIT_CLK_HZ <= MAX_RATE_BPS);

   put_str("sport_fpga_tx_prbs_long bytes=");
   put_u64(bytes);
   put_str(" words=");
   put_u32(got_words);
   put_str(" errors=");
   put_u32(errors);
   put_str(" firsterr=");
   put_i32(firsterr);
   put_str(" firstgot=0x");
   put_hex32(firstgot);
   put_str(" firstexp=0x");
   put_hex32(firstexp);
   put_str(" timeouts=");
   put_u32(timeouts);
   put_str(" overruns=");
   put_u32(overruns);
   put_str(" wrap_misses=");
   put_u32(wrap_misses);
   put_str(" sport_error=");
   put_u32(sport_error ? 1U : 0U);
   put_str(" sport_err=0x");
   put_hex32(sport_err);
   put_str(" bit_clk_hz=");
   put_u32(BIT_CLK_HZ);
   put_str(" ticks=");
   put_u64(elapsed_ticks64);
   put_str(" rate_bps=");
   put_u64(rate_bps);
   put_str(" ");
   put_str(pass ? "PASS\r\n" : "FAIL\r\n");
   if (err_saved != 0U) {
      put_str("sport_fpga_tx_errs");
      for (uint32_t i = 0U; i < err_saved; i++) {
         put_str(" idx=");
         put_u32(err_idx[i]);
         put_str(" got=0x");
         put_hex32(err_got[i]);
         put_str(" exp=0x");
         put_hex32(err_exp[i]);
      }
      put_str("\r\n");
   }

   for (;;) {
   }
}
