// SPDX-License-Identifier: MIT
// main.c --- P13/P14 GPIO connectivity tester for the ADSP-21569
// Copyright (c) 2026 Jakob Kastelic

// Finds pairs of header pins that are shorted together. For
// each testable pin the demo drives it high, samples every
// other testable pin, then drives it low and samples again.
// The full NxN observation matrix is kept so symmetric pairs
// can be deduplicated and one-way anomalies surfaced.
//
// P13 hosts DAI1 pin buffers (the DAI0 slots on P13 are not
// bonded on either package so they are omitted). P14 hosts
// PORTA/PORTB signals; PA06/PA07 carry UART0 TX/RX and are
// deliberately excluded from the scan so the diagnostic UART
// is never disturbed -- this also keeps the report stream
// clean instead of having a torn-up window during the probe
// phase. DAI1_PIN11/PIN12 appear on P13.22/P13.24 but are only
// bonded on the 400-ball BGA package, so they are omitted to
// stay safe on 120-lead LQFP.
//
// Pins with no internal pull-down float in the tri-stated
// baseline read and are excluded from anomaly reports so the
// noise does not drown out real findings; genuine shorts
// still appear because the target tracks the driven source
// deterministically in both states.
//
// Report runs once at boot. After that the demo accepts a few
// single-character commands on UART0:
//   t           re-run the connectivity scan
//   ?  or  h    print the command list and the pin index table
//   b<idx>      blink a single pin (idx is one hex-ish nibble:
//               '0'..'9' for pins 0..9, 'a'..'n' for pins 10..23)
//
// Bench connectivity test commands (no <idx> argument; each one
// references an internal cursor maintained by `R`/`Z`):
//   N           drive cursor pin as output high
//   n           drive cursor pin as output low (same pin as `N`)
//   R           park cursor pin back as input and advance cursor
//   Z           reset cursor to 0 and park every pin as input
//   Q           emit "Q=<6 hex digits>\r\n" snapshot of all 22 pins
//               (LSB = pin 0; bit i = 1 means pin i reads high)

#include "board.h"
#include "clocks.h"
#include "gpio.h"
#include "timer.h"
#include "uart.h"
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>

// cc21k printf mishandles variadic %u/%s on this build, so this
// file uses non-variadic diag_* helpers (same pattern as qspi).
static void diag_puts(const char *s)
{
   while (*s)
      uart_putc(*s++);
}

static void diag_dec(uint32_t v)
{
   static const uint32_t pow10[] = {
       1000000000U, 100000000U, 10000000U, 1000000U, 100000U,
       10000U,      1000U,      100U,      10U,      1U};
   bool emitted = false;
   for (unsigned i = 0; i < sizeof(pow10) / sizeof(pow10[0]); i++) {
      uint32_t digit = 0;
      while (v >= pow10[i]) {
         v -= pow10[i];
         digit++;
      }
      if (digit != 0 || emitted || pow10[i] == 1U) {
         uart_putc((char)('0' + digit));
         emitted = true;
      }
   }
}

#define SETTLE_MS 2U

struct pin_entry {
   const char *header; // e.g. "P13.02"
   const char *signal; // e.g. "DAI1_PIN01"
   enum gpio_pin pin;
   enum gpio_bank bnk;
   uint32_t stat_bit;
};

static const struct pin_entry pins[] = {
    // P13 -- DAI1 pin buffers (DAI0 slots and DAI1_13..18 not
    // bonded; DAI1_11/12 bonded only on BGA and so skipped).
    {"P13.02", "DAI1_PIN01", GPIO_DAI1_01, GPIO_BANK_DAI1,  0U },
    {"P13.04", "DAI1_PIN02", GPIO_DAI1_02, GPIO_BANK_DAI1,  1U },
    {"P13.06", "DAI1_PIN03", GPIO_DAI1_03, GPIO_BANK_DAI1,  2U },
    {"P13.08", "DAI1_PIN04", GPIO_DAI1_04, GPIO_BANK_DAI1,  3U },
    {"P13.10", "DAI1_PIN05", GPIO_DAI1_05, GPIO_BANK_DAI1,  4U },
    {"P13.12", "DAI1_PIN06", GPIO_DAI1_06, GPIO_BANK_DAI1,  5U },
    {"P13.14", "DAI1_PIN07", GPIO_DAI1_07, GPIO_BANK_DAI1,  6U },
    {"P13.16", "DAI1_PIN08", GPIO_DAI1_08, GPIO_BANK_DAI1,  7U },
    {"P13.18", "DAI1_PIN09", GPIO_DAI1_09, GPIO_BANK_DAI1,  8U },
    {"P13.20", "DAI1_PIN10", GPIO_DAI1_10, GPIO_BANK_DAI1,  9U },
    {"P13.38", "DAI1_PIN19", GPIO_DAI1_19, GPIO_BANK_DAI1,  18U},
    {"P13.40", "DAI1_PIN20", GPIO_DAI1_20, GPIO_BANK_DAI1,  19U},

    // P14 -- PORTA/PORTB.  PA06/PA07 carry UART0 TX/RX and are
    // intentionally absent from this table.
    {"P14.06", "PA_08",      GPIO_PA08,    GPIO_BANK_PORTA, 8U },
    {"P14.08", "PA_09",      GPIO_PA09,    GPIO_BANK_PORTA, 9U },
    {"P14.10", "PB_05",      GPIO_PB05,    GPIO_BANK_PORTB, 5U },
    {"P14.12", "PA_10",      GPIO_PA10,    GPIO_BANK_PORTA, 10U},
    {"P14.14", "PA_11",      GPIO_PA11,    GPIO_BANK_PORTA, 11U},
    {"P14.16", "PA_12",      GPIO_PA12,    GPIO_BANK_PORTA, 12U},
    {"P14.18", "PA_13",      GPIO_PA13,    GPIO_BANK_PORTA, 13U},
    {"P14.20", "PB_10",      GPIO_PB10,    GPIO_BANK_PORTB, 10U},
    {"P14.25", "PA_14",      GPIO_PA14,    GPIO_BANK_PORTA, 14U},
    {"P14.27", "PA_15",      GPIO_PA15,    GPIO_BANK_PORTA, 15U},
};

#define N_PINS 22U

static_assert(sizeof(pins) / sizeof(pins[0]) == N_PINS,
              "N_PINS out of sync with pin table");

static const char *port_name(enum gpio_bank b)
{
   switch (b) {
      case GPIO_BANK_PORTA: return "PORTA";
      case GPIO_BANK_PORTB: return "PORTB";
      case GPIO_BANK_PORTC: return "PORTC";
      case GPIO_BANK_DAI0: return "DAI0";
      case GPIO_BANK_DAI1: return "DAI1";
      default: return "?";
   }
}

// Format: P13.02 (DAI1_PIN01 on DAI1).
static void print_pin(uint32_t idx)
{
   diag_puts(pins[idx].header);
   diag_puts(" (");
   diag_puts(pins[idx].signal);
   diag_puts(" on ");
   diag_puts(port_name(pins[idx].bnk));
   diag_puts(")");
}

// Per-source/target observation codes.
#define OBS_ISOLATED 0U // hi:0 lo:0 -- target never saw the drive
#define OBS_FOLLOW   1U // hi:1 lo:0 -- clean short
#define OBS_INVERT   2U // hi:0 lo:1 -- inverted (buffer? transistor?)
#define OBS_STUCK_HI 3U // hi:1 lo:1 -- stuck high while source toggles

static uint8_t obs[N_PINS][N_PINS]; // obs[src][tgt]

static uint32_t sample_bit(const uint32_t banks[GPIO_BANK_COUNT], uint32_t idx)
{
   return (banks[pins[idx].bnk] >> pins[idx].stat_bit) & 1U;
}

static void snapshot(uint32_t banks[GPIO_BANK_COUNT])
{
   banks[GPIO_BANK_PORTA] = gpio_read_bank(GPIO_BANK_PORTA);
   banks[GPIO_BANK_PORTB] = gpio_read_bank(GPIO_BANK_PORTB);
   banks[GPIO_BANK_PORTC] = gpio_read_bank(GPIO_BANK_PORTC);
   banks[GPIO_BANK_DAI0]  = gpio_read_bank(GPIO_BANK_DAI0);
   banks[GPIO_BANK_DAI1]  = gpio_read_bank(GPIO_BANK_DAI1);
}

static uint8_t classify(uint32_t h, uint32_t l)
{
   if (h == 1U && l == 0U)
      return OBS_FOLLOW;
   if (h == 0U && l == 1U)
      return OBS_INVERT;
   if (h == 1U && l == 1U)
      return OBS_STUCK_HI;
   return OBS_ISOLATED;
}

static void probe_source(uint32_t src)
{
   uint32_t hi_banks[GPIO_BANK_COUNT];
   uint32_t lo_banks[GPIO_BANK_COUNT];

   gpio_make_output(pins[src].pin);
   gpio_write(pins[src].pin, true);
   delay_ms(SETTLE_MS);
   snapshot(hi_banks);

   gpio_write(pins[src].pin, false);
   delay_ms(SETTLE_MS);
   snapshot(lo_banks);

   gpio_make_input(pins[src].pin);

   for (uint32_t t = 0; t < N_PINS; t++) {
      if (t == src) {
         obs[src][t] = OBS_ISOLATED;
         continue;
      }
      obs[src][t] = classify(sample_bit(hi_banks, t), sample_bit(lo_banks, t));
   }
}

// Drop every pin into the high-impedance state and record
// which of them still reads high. Pins without an internal
// pull-down float in that state and their observations in the
// per-source sweep cannot be trusted; the caller uses this
// mask to suppress their anomaly reports.
static void measure_floating(bool floating[N_PINS])
{
   uint32_t baseline[GPIO_BANK_COUNT];
   for (uint32_t i = 0; i < N_PINS; i++) {
      gpio_make_input(pins[i].pin);
   }
   delay_ms(SETTLE_MS);
   snapshot(baseline);
   for (uint32_t i = 0; i < N_PINS; i++) {
      floating[i] = sample_bit(baseline, i) != 0U;
   }
}

// Symmetric follows -- clean shorts. Print each pair once.
static void report_pairs(void)
{
   for (uint32_t i = 0; i < N_PINS; i++) {
      for (uint32_t j = i + 1; j < N_PINS; j++) {
         if (obs[i][j] == OBS_FOLLOW && obs[j][i] == OBS_FOLLOW) {
            diag_puts("connected: ");
            print_pin(i);
            diag_puts(" <-> ");
            print_pin(j);
            diag_puts("\r\n");
         }
      }
   }
}

// Asymmetric follows -- one direction tracks but the reverse
// does not. Skip pairs involving a floating pin because its
// reverse reading is unreliable.
static void report_one_way(const bool floating[N_PINS])
{
   for (uint32_t i = 0; i < N_PINS; i++) {
      for (uint32_t j = 0; j < N_PINS; j++) {
         if (i == j || floating[i] || floating[j])
            continue;
         if (obs[i][j] == OBS_FOLLOW && obs[j][i] != OBS_FOLLOW) {
            diag_puts("WARN one-way follow: ");
            print_pin(i);
            diag_puts(" -> ");
            print_pin(j);
            diag_puts("\r\n");
         }
      }
   }
}

// Inverted or stuck-high observations on non-floating targets.
static void report_weird(const bool floating[N_PINS])
{
   for (uint32_t i = 0; i < N_PINS; i++) {
      for (uint32_t j = 0; j < N_PINS; j++) {
         if (i == j || floating[j])
            continue;
         if (obs[i][j] == OBS_INVERT) {
            diag_puts("WARN inverted: drive ");
            print_pin(i);
            diag_puts(", ");
            print_pin(j);
            diag_puts(" reads opposite\r\n");
         } else if (obs[i][j] == OBS_STUCK_HI) {
            diag_puts("WARN stuck-high: drive ");
            print_pin(i);
            diag_puts(", ");
            print_pin(j);
            diag_puts(" stays high\r\n");
         }
      }
   }
}

// Pin indices encode as one ASCII char: '0'..'9' for 0..9 then
// 'a'.. for 10.. (case-insensitive on parse).
#define IDX_DIGIT_SPAN 10U

static char idx_to_char(uint32_t idx)
{
   return (char)(idx < IDX_DIGIT_SPAN ? ('0' + idx)
                                      : ('a' + (idx - IDX_DIGIT_SPAN)));
}

static int parse_pin_idx(int c)
{
   if (c >= '0' && c <= '9')
      return c - '0';
   if (c >= 'a' && c <= 'z')
      return (int)IDX_DIGIT_SPAN + (c - 'a');
   if (c >= 'A' && c <= 'Z')
      return (int)IDX_DIGIT_SPAN + (c - 'A');
   return -1;
}

#define BLINK_HALF_MS 250U
#define BLINK_TOGGLES 10U

static void blink_pin(uint32_t idx)
{
   if (idx >= N_PINS) {
      diag_puts("bad pin idx\r\n");
      return;
   }
   diag_puts("blinking ");
   uart_putc(idx_to_char(idx));
   diag_puts(" = ");
   print_pin(idx);
   diag_puts("\r\n");
   gpio_make_output(pins[idx].pin);
   for (uint32_t i = 0; i < BLINK_TOGGLES; i++) {
      gpio_write(pins[idx].pin, true);
      delay_ms(BLINK_HALF_MS);
      gpio_write(pins[idx].pin, false);
      delay_ms(BLINK_HALF_MS);
   }
   gpio_make_input(pins[idx].pin);
   diag_puts("blink done\r\n");
}

static void print_help(void)
{
   diag_puts("commands:\r\n");
   diag_puts("  t        run connectivity scan\r\n");
   diag_puts("  ?  / h   print this help\r\n");
   diag_puts("  b<idx>   blink one pin for a few seconds\r\n");
   diag_puts("  N/n/R/Z  bench cursor: drive hi / lo / release+advance / reset\r\n");
   diag_puts("  Q        emit pin-state hex snapshot\r\n");
   diag_puts("pin index table:\r\n");
   for (uint32_t i = 0; i < N_PINS; i++) {
      diag_puts("  ");
      uart_putc(idx_to_char(i));
      diag_puts("  ");
      print_pin(i);
      diag_puts("\r\n");
   }
}

// ---- bench-driven per-pin cursor --------------------------------
//
// The mission file iterates through 16 pins via run.py's `Foreach
// count(16)` loop and sends the same fixed-text command sequence
// each iteration; the firmware therefore needs an internal cursor
// to know which pin a context-free `N`/`n`/`R` byte refers to.
// The cursor is initialised to 0 by `Z` and advanced one position
// by every `R`. Both ends of the bench fixture (DSP and FPGA)
// implement the same protocol so the plan never has to name a pin
// directly.

static uint32_t cursor;

static void emit_pin_snapshot(void)
{
   // Force every pin back to input before sampling so that a Q
   // arriving while some pin (the cursor pin in Phase A, a stale
   // boot-scan source, etc.) is still configured as an output
   // does not just read back its own driven data register. The
   // call is idempotent on pins already in input mode -- it
   // re-asserts INEN / PADS_DAI_IE and clears DIR / PBEN -- so
   // running it on every Q has no side effects when the caller
   // has already parked everything. The crucial property is that
   // the value emitted reflects the EXTERNAL pin level, never
   // the cached output drive.
   for (uint32_t i = 0; i < N_PINS; i++) {
      gpio_make_input(pins[i].pin);
   }
   uint32_t banks[GPIO_BANK_COUNT];
   snapshot(banks);
   uint32_t v = 0U;
   for (uint32_t i = 0; i < N_PINS; i++) {
      if (sample_bit(banks, i)) {
         v |= (1U << i);
      }
   }
   diag_puts("Q=");
   for (int nib = 5; nib >= 0; nib--) {
      uint32_t n = (v >> (nib * 4)) & 0xFU;
      uart_putc((char)(n < 10 ? ('0' + n) : ('a' + (n - 10))));
   }
   diag_puts("\r\n");
}

static void park_all_inputs(void)
{
   for (uint32_t i = 0; i < N_PINS; i++) {
      gpio_make_input(pins[i].pin);
   }
}

static void run_test(uint32_t cycle)
{
   diag_puts("=== scan ");
   diag_dec(cycle);
   diag_puts(" ===\r\n");

   static bool floating[N_PINS];
   measure_floating(floating);
   for (uint32_t i = 0; i < N_PINS; i++) {
      probe_source(i);
   }
   // Leave every probed pin parked as input.
   for (uint32_t i = 0; i < N_PINS; i++) {
      gpio_make_input(pins[i].pin);
   }

   report_pairs();
   report_one_way(floating);
   report_weird(floating);

   // Post-restore marker: UART is clean by this point so the line is
   // safe to grep, unlike the opening "=== scan N ===" banner whose
   // first byte rides through the probe phase.
   diag_puts("=== scan ");
   diag_dec(cycle);
   diag_puts(" done ===\r\n");
}

int main(void)
{
   static const struct clocks_cfg clk = BOARD_CLOCKS_CFG;
   clocks_init(&clk);
   uart_init(BOARD_BAUD_DIV);
   timer_init();
   board_som_init(0U);

   for (uint32_t i = 0; i < N_PINS; i++) {
      gpio_make_input(pins[i].pin);
   }

   uint32_t cycle = 0;
   run_test(cycle++);

   for (;;) {
      int c = uart_try_getc();
      if (c < 0) {
         continue;
      }
      if (c == 't' || c == 'T') {
         run_test(cycle++);
      } else if (c == '?' || c == 'h') {
         print_help();
      } else if (c == 'b' || c == 'B') {
         int arg = -1;
         while (arg < 0) {
            arg = uart_try_getc();
         }
         int idx = parse_pin_idx(arg);
         if (idx < 0) {
            diag_puts("bad pin idx\r\n");
         } else {
            blink_pin((uint32_t)idx);
         }
      } else if (c == 'N') {
         if (cursor < N_PINS) {
            gpio_make_output(pins[cursor].pin);
            gpio_write(pins[cursor].pin, true);
         }
         // Echo cursor + self-readback of the driven bit so the
         // bench can tell whether the pad actually reaches HIGH
         // from the DSP's own perspective; if the FPGA disagrees,
         // the discrepancy is downstream of the DSP drive.
         diag_puts("N");
         uart_putc(idx_to_char(cursor < N_PINS ? cursor : 0U));
         if (cursor < N_PINS) {
            uint32_t banks[GPIO_BANK_COUNT];
            snapshot(banks);
            diag_puts(sample_bit(banks, cursor) ? "=1" : "=0");
         }
         diag_puts("\r\n");
      } else if (c == 'n') {
         if (cursor < N_PINS) {
            gpio_make_output(pins[cursor].pin);
            gpio_write(pins[cursor].pin, false);
         }
         diag_puts("n");
         uart_putc(idx_to_char(cursor < N_PINS ? cursor : 0U));
         if (cursor < N_PINS) {
            uint32_t banks[GPIO_BANK_COUNT];
            snapshot(banks);
            diag_puts(sample_bit(banks, cursor) ? "=1" : "=0");
         }
         diag_puts("\r\n");
      } else if (c == 'R') {
         if (cursor < N_PINS) {
            gpio_make_input(pins[cursor].pin);
         }
         cursor++;
      } else if (c == 'Z') {
         park_all_inputs();
         cursor = 0U;
      } else if (c == 'Q') {
         emit_pin_snapshot();
      }
   }
}
