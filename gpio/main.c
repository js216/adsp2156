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
// PORTA/PORTB signals; PA06/PA07 carry UART0 TX/RX and must
// not be driven while printf is talking, so they are skipped.
// DAI1_PIN11/PIN12 appear on P13.22/P13.24 but are only
// bonded on the 400-ball BGA package, so they are omitted to
// stay safe on 120-lead LQFP.
//
// Pins with no internal pull-down float in the tri-stated
// baseline read and are excluded from anomaly reports so the
// noise does not drown out real findings; genuine shorts
// still appear because the target tracks the driven source
// deterministically in both states.
//
// Report runs once at boot and again every time a 't' arrives
// on UART0.

#include "board.h"
#include "clocks.h"
#include "gpio.h"
#include "timer.h"
#include "uart.h"
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#define SETTLE_MS 2U

struct pin_entry {
   const char    *header; // e.g. "P13.02"
   const char    *signal; // e.g. "DAI1_PIN01"
   enum gpio_pin  pin;
   enum gpio_bank bnk;
   uint32_t       stat_bit;
};

static const struct pin_entry pins[] = {
    // P13 -- DAI1 pin buffers (DAI0 slots and DAI1_13..18 not
    // bonded; DAI1_11/12 bonded only on BGA and so skipped).
    {"P13.02", "DAI1_PIN01", GPIO_DAI1_01, GPIO_BANK_DAI1, 0U},
    {"P13.04", "DAI1_PIN02", GPIO_DAI1_02, GPIO_BANK_DAI1, 1U},
    {"P13.06", "DAI1_PIN03", GPIO_DAI1_03, GPIO_BANK_DAI1, 2U},
    {"P13.08", "DAI1_PIN04", GPIO_DAI1_04, GPIO_BANK_DAI1, 3U},
    {"P13.10", "DAI1_PIN05", GPIO_DAI1_05, GPIO_BANK_DAI1, 4U},
    {"P13.12", "DAI1_PIN06", GPIO_DAI1_06, GPIO_BANK_DAI1, 5U},
    {"P13.14", "DAI1_PIN07", GPIO_DAI1_07, GPIO_BANK_DAI1, 6U},
    {"P13.16", "DAI1_PIN08", GPIO_DAI1_08, GPIO_BANK_DAI1, 7U},
    {"P13.18", "DAI1_PIN09", GPIO_DAI1_09, GPIO_BANK_DAI1, 8U},
    {"P13.20", "DAI1_PIN10", GPIO_DAI1_10, GPIO_BANK_DAI1, 9U},
    {"P13.38", "DAI1_PIN19", GPIO_DAI1_19, GPIO_BANK_DAI1, 18U},
    {"P13.40", "DAI1_PIN20", GPIO_DAI1_20, GPIO_BANK_DAI1, 19U},

    // P14 -- PORTA/PORTB. PA06/PA07 skipped (UART0 TX/RX).
    {"P14.06", "PA_08", GPIO_PA08, GPIO_BANK_PORTA, 8U},
    {"P14.08", "PA_09", GPIO_PA09, GPIO_BANK_PORTA, 9U},
    {"P14.10", "PB_05", GPIO_PB05, GPIO_BANK_PORTB, 5U},
    {"P14.12", "PA_10", GPIO_PA10, GPIO_BANK_PORTA, 10U},
    {"P14.14", "PA_11", GPIO_PA11, GPIO_BANK_PORTA, 11U},
    {"P14.16", "PA_12", GPIO_PA12, GPIO_BANK_PORTA, 12U},
    {"P14.18", "PA_13", GPIO_PA13, GPIO_BANK_PORTA, 13U},
    {"P14.20", "PB_10", GPIO_PB10, GPIO_BANK_PORTB, 10U},
    {"P14.25", "PA_14", GPIO_PA14, GPIO_BANK_PORTA, 14U},
    {"P14.27", "PA_15", GPIO_PA15, GPIO_BANK_PORTA, 15U},
};

#define N_PINS 22U

// Catch pin-table size drift at compile time without relying
// on _Static_assert (cc21k rejects it): a negative-size array
// typedef fails to compile when the equation is false.
typedef char n_pins_size_check[(sizeof(pins) / sizeof(pins[0]) == N_PINS) ? 1 : -1];

static const char *port_name(enum gpio_bank b)
{
   switch (b) {
      case GPIO_BANK_PORTA: return "PORTA";
      case GPIO_BANK_PORTB: return "PORTB";
      case GPIO_BANK_PORTC: return "PORTC";
      case GPIO_BANK_DAI0:  return "DAI0";
      case GPIO_BANK_DAI1:  return "DAI1";
      default:              return "?";
   }
}

// Format: P13.02 (DAI1_PIN01 on DAI1).
static void print_pin(uint32_t idx)
{
   printf("%s (%s on %s)", pins[idx].header, pins[idx].signal,
          port_name(pins[idx].bnk));
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
   if (h == 1U && l == 0U) return OBS_FOLLOW;
   if (h == 0U && l == 1U) return OBS_INVERT;
   if (h == 1U && l == 1U) return OBS_STUCK_HI;
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

static void run_test(uint32_t cycle)
{
   printf("=== scan %u ===\r\n", cycle);

   uint32_t baseline[GPIO_BANK_COUNT];
   for (uint32_t i = 0; i < N_PINS; i++) {
      gpio_make_input(pins[i].pin);
   }
   delay_ms(SETTLE_MS);
   snapshot(baseline);
   static bool floating[N_PINS];
   for (uint32_t i = 0; i < N_PINS; i++) {
      floating[i] = sample_bit(baseline, i) != 0U;
   }

   for (uint32_t i = 0; i < N_PINS; i++) {
      probe_source(i);
   }

   // Symmetric follows -- clean shorts. Print each pair once.
   for (uint32_t i = 0; i < N_PINS; i++) {
      for (uint32_t j = i + 1; j < N_PINS; j++) {
         if (obs[i][j] == OBS_FOLLOW && obs[j][i] == OBS_FOLLOW) {
            printf("connected: ");
            print_pin(i);
            printf(" <-> ");
            print_pin(j);
            printf("\r\n");
         }
      }
   }

   // Asymmetric follows -- one direction tracks but the reverse
   // does not. Skip pairs involving a floating pin because its
   // reverse reading is unreliable.
   for (uint32_t i = 0; i < N_PINS; i++) {
      for (uint32_t j = 0; j < N_PINS; j++) {
         if (i == j || floating[i] || floating[j]) continue;
         if (obs[i][j] == OBS_FOLLOW && obs[j][i] != OBS_FOLLOW) {
            printf("WARN one-way follow: ");
            print_pin(i);
            printf(" -> ");
            print_pin(j);
            printf("\r\n");
         }
      }
   }

   // Inverted or stuck-high observations on non-floating
   // targets.
   for (uint32_t i = 0; i < N_PINS; i++) {
      for (uint32_t j = 0; j < N_PINS; j++) {
         if (i == j || floating[j]) continue;
         if (obs[i][j] == OBS_INVERT) {
            printf("WARN inverted: drive ");
            print_pin(i);
            printf(", ");
            print_pin(j);
            printf(" reads opposite\r\n");
         } else if (obs[i][j] == OBS_STUCK_HI) {
            printf("WARN stuck-high: drive ");
            print_pin(i);
            printf(", ");
            print_pin(j);
            printf(" stays high\r\n");
         }
      }
   }
}

int main(void)
{
   static const struct clocks_cfg clk = BOARD_CLOCKS_CFG;
   clocks_init(&clk);
   uart_init(BOARD_BAUD_DIV);
   timer_init();

   for (uint32_t i = 0; i < N_PINS; i++) {
      gpio_make_input(pins[i].pin);
   }

   uint32_t cycle = 0;
   run_test(cycle++);

   for (;;) {
      int c = uart_try_getc();
      if (c == 't' || c == 'T') {
         run_test(cycle++);
      }
   }
}
