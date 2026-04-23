// SPDX-License-Identifier: MIT
// main.c --- SPI2 interactive hex-dump shell
// Copyright (c) 2026 Jakob Kastelic

// UART0 is a tiny interactive shell that configures SPI2 as a slave
// and drains receive data as raw hex.  No PRBS, no loopback, no
// master mode.  Intended for bring-up: point FT4222 (master) at
// SPI2, open a terminal on UART0, type commands, watch words.
//
// Commands (see `help` for the live list):
//   help | ?           print command list + current state
//   m1 | m2 | m4       lane width: 1 (default), 2, 4
//   b <bytes>          DMA buffer size
//   a 0 | a 1          auto-consume off/on (default on)
//   i                  drain buffer, print level + running checksum,
//                      reset checksum
//   h                  stream received words forever; any UART byte
//                      stops the stream
//   h <n>              stream exactly <n> bytes (multiple of 4)
//
// Shell features:
//   * Characters echo as typed.  CR / LF commit a line.
//   * Backspace (0x08 or 0x7F) edits the line.
//   * Up-arrow (ESC [ A) recalls the most recent command; one-slot
//     history (previous line only).
//   * Lines longer than CMD_BUF_SIZE are truncated silently.
//
// Pin mapping (PA0..PA5, alternate function "b"):
//   PA0 = SPI2_MISO / D1
//   PA1 = SPI2_MOSI / D0
//   PA2 = SPI2_D2
//   PA3 = SPI2_D3
//   PA4 = SPI2_CLK
//   PA5 = SPI2_SEL1
//
// SPI is always configured CPOL=0, CPHA=1, MSB-first, 32-bit word
// size.  Received words print as `w<idx>=0x<hex>`.

#include "board.h"
#include "clocks.h"
#include "dma.h"
#include "pinmux.h"
#include "regs.h"
#include "spi.h"
#include "timer.h"
#include "uart.h"
#include <stdbool.h>
#include <stdint.h>

#define SPI_PORT       SPI_ID_2
#define SPI_RX_DMA     DMA_CH_SPI2_RX
#define SPI_TX_DMA     DMA_CH_SPI2_TX
#define SPI_STRIDE     0x1000U
#define STARTUP_MS     500U
#define BYTES_PER_WORD 4U
#define BITS_PER_BYTE  8U
#define BYTE_MASK      0xFFU
#define CMD_BUF_SIZE   128U
#define BASE_DEC       10U
#define BASE_HEX       16U
// Ping-pong minimum ring size: two 32-bit words (one per half).
#define PP_MIN_BYTES (2U * BYTES_PER_WORD)

// DMA receive buffer, in 32-bit words.  Ping-pong: split into two
// equal halves.  DMA fills half 0, IRQDONE latches, DMA fills half 1
// (via descriptor chain), IRQDONE re-latches, back to half 0 -- for
// ever.  CPU drains each half after its IRQDONE arrives while the
// DMA writes the other half; the two sides never touch the same
// half at the same time, so no race on the ring buffer contents.
//
// Landed in L2 SRAM via seg_l2_bss -- L1 block1 faults the core at
// reset past ~32 KB of initialised data.  L2 has ~1 MiB total; 512
// KiB here leaves headroom for other seg_l2_bss data.
#define DMA_BUF_WORDS (512U * 1024U / BYTES_PER_WORD)

#pragma section("seg_l2_bss", NO_INIT)
static uint32_t dma_rx_buf[DMA_BUF_WORDS];

// Ping-pong descriptor pair.  Force into L2 so the DDE can fetch
// via its identity-mapped system alias -- L1 placements would need
// the L1_SYS_OFFSET translation and only for the L1_INT window.
#pragma section("seg_l2_bss", NO_INIT)
static struct dma_dscl dma_desc[2];

// TX DMA buffer for slave-mode outbound streaming.  256 KiB fits
// alongside the 512 KiB RX ring in L2 with margin for the rest of
// seg_l2_bss.  Longer TX tests use DMA FLOW=AUTO on this buffer
// so the pattern repeats forever; a one-shot (FLOW=STOP) variant
// sends exactly ``n`` words then stops.
#define DMA_TX_WORDS (256U * 1024U / BYTES_PER_WORD)
#pragma section("seg_l2_bss", NO_INIT)
static uint32_t dma_tx_buf[DMA_TX_WORDS];

// Host-compatible 32-bit xorshift PRBS.  Matches
// make_qspi.prbs_xorshift32 / QspiTest._prbs byte-for-byte: each
// call updates state then emits the low 8 bits as the next byte.
// Used by the TX path to generate streams the poller can verify
// via q.read_verify_prbs.  Seed must be non-zero.
static uint32_t prbs_state;

static void prbs_init(uint32_t seed)
{
   prbs_state = (seed == 0U) ? 1U : seed;
}

// xorshift32 tap positions, matching the host generator.
#define PRBS_TAP_A 13U
#define PRBS_TAP_B 17U
#define PRBS_TAP_C 5U

static uint8_t prbs_next_byte(void)
{
   uint32_t x = prbs_state;
   x ^= (uint32_t)(x << PRBS_TAP_A);
   x ^= (uint32_t)(x >> PRBS_TAP_B);
   x ^= (uint32_t)(x << PRBS_TAP_C);
   prbs_state = x;
   return (uint8_t)(x & BYTE_MASK);
}

// ---------- diag_* : non-variadic output helpers ----------
// printf() on cc21k mis-handles single-arg variadic slots (observed
// `%s` returning empty, `%x` reading wrong register).  Build each
// line via putchar to sidestep va_arg.
static void diag_puts(const char *s)
{
   while (*s)
      uart_putc(*s++);
}

#define DIAG_NIBBLE_MASK   0xFU
#define DIAG_NIBBLE_BITS   4U
#define DIAG_HEX_TOP_SHIFT 28
#define DIAG_DEC_RADIX     10U

static void diag_hex32(uint32_t v)
{
   for (int shift = DIAG_HEX_TOP_SHIFT; shift >= 0;
        shift -= (int)DIAG_NIBBLE_BITS) {
      uint32_t n = (v >> (unsigned)shift) & DIAG_NIBBLE_MASK;
      uart_putc(
          (char)(n < DIAG_DEC_RADIX ? '0' + n : 'a' + (n - DIAG_DEC_RADIX)));
   }
}

// ---------- SPI config state ----------
static uint32_t spi_base;
static enum spi_miom current_miom = SPI_MIO_SINGLE;
// Active DMA ring size in 32-bit words.  Capped by the static
// dma_rx_buf[] allocation (DMA_BUF_WORDS).  b command adjusts it.
// dma_half_words = dma_active_words >> 1 is the per-descriptor XCNT
// (pre-computed so the DMA/drain paths avoid any u32 divide, which
// the -no-std-lib build can't link against libc's __divrem_u32).
static uint32_t dma_active_words = DMA_BUF_WORDS;
static uint32_t dma_half_words   = DMA_BUF_WORDS >> 1U;
// Running XOR-checksum + word count accumulated since the last
// `i` command (which prints and resets them).  No auto-reset on
// buffer fill -- if the ring wraps faster than the host drains,
// data is lost silently.
static uint32_t cksum_xor   = 0;
static uint32_t cksum_count = 0;
// Always-on DMA ring bookkeeping.  Armed at boot in slave role so
// data received before `h` is already in the ring.  Ping-pong
// rhythm: drain_poll_new_words compares the half rd_pos is in
// against the half DMA_ADDR_CUR is in; when they differ, the
// consumer's half has been fully written and those words are
// ready to consume.
static bool dma_armed      = false;
static uint32_t dma_rd_pos = 0;
// TX (slave transmit) DMA state.  tx_armed tracks whether the
// SPI2 TX peripheral DMA channel is currently running.  Separate
// from dma_armed (which is RX) -- they're never both active in
// the slave-only demo because pinmux/FER direction must flip and
// the SPI peripheral only drives one direction at a time.
static bool tx_armed = false;
// Auto-consume: when true, the shell's idle-wait loop continuously
// samples the DMA ring and folds new words into the running
// checksum.  This keeps up with DMA on bursts larger than the ring
// size (drain_poll_new_words only handles one wrap per sample, so
// without periodic sampling during a long transfer, data is lost
// when the ring wraps).  When false, words accumulate in the ring
// until `i` runs -- bursts above ring size lose data silently.
static bool auto_consume = true;

static void drain_disarm(void);
static void drain_arm(void);
static uint32_t drain_poll_new_words(void);
static void tx_disarm(void);

static const char *miom_name(enum spi_miom m)
{
   // Explicit if-chain: cc21k's switch lowering produced a NULL-return
   // path that printed as "(null)" on UART.
   static const char s_x1[] = "x1";
   static const char s_x2[] = "x2";
   static const char s_x4[] = "x4";
   static const char s_qq[] = "??";
   unsigned v               = (unsigned)m;
   if (v == (unsigned)SPI_MIO_SINGLE)
      return s_x1;
   if (v == (unsigned)SPI_MIO_DUAL)
      return s_x2;
   if (v == (unsigned)SPI_MIO_QUAD)
      return s_x4;
   return s_qq;
}

static void spi_reconfigure(enum spi_miom miom)
{
   // Slave-only demo.  CPOL=0, CPHA=1 matches pyft4222
   // Cpha.CLK_TRAILING (flags=0 in poller.py).
   struct spi_cfg cfg = {
       .clkdiv    = 0,
       .size      = SPI_WORD_32,
       .miom      = miom,
       .is_master = 0U,
       .cpol      = 0,
       .cpha      = 1U,
       .lsb_first = 0,
   };
   drain_disarm();
   tx_disarm();
   // HRM 15-12: "Changing to quad SPI mode must be done when the
   // SPI is in a quiescent state."  spi_disable polls SPIF + RFE
   // and then clears CTL.EN before pinmux_spi2 toggles FER bits;
   // changing pin function while the SPI shifter is active drops
   // the opening bytes of the first transfer in the new mode.
   spi_disable(SPI_PORT);
   pinmux_spi2(0, (unsigned)miom);
   spi_init(SPI_PORT, &cfg);
   spi_rx_dma_enable(SPI_PORT);
   current_miom = miom;
   // Arm DMA before enabling the SPI RX channel so the DDE is
   // already in data-transfer state by the time the first word
   // can hit the 2-deep RFIFO.
   drain_arm();
   spi_rx_enable(SPI_PORT);
}

// Switch SPI2 into slave-transmit direction at the current
// lane width.  Pin FER flips so the data lanes become DSP
// outputs (single: PA0; dual: PA0..PA1; quad: PA0..PA3); RX DMA
// is disarmed; TX peripheral DMA line is enabled.  Caller must
// have pre-filled dma_tx_buf and then call tx_arm(n_words) to
// start transmission.
static void spi_reconfigure_tx(enum spi_miom miom)
{
   struct spi_cfg cfg = {
       .clkdiv    = 0,
       .size      = SPI_WORD_32,
       .miom      = miom,
       .is_master = 0U,
       .cpol      = 0,
       .cpha      = 1U,
       .lsb_first = 0,
   };
   drain_disarm();
   tx_disarm();
   spi_disable(SPI_PORT);
   pinmux_spi2_dir(0, (unsigned)miom, 1);
   spi_init(SPI_PORT, &cfg);
   spi_tx_dma_enable(SPI_PORT);
   current_miom = miom;
}

// One-shot TX arm: configure the SPI2 TX DMA channel (FLOW=STOP)
// to stream n_words from dma_tx_buf into the peripheral's TFIFO,
// then enable the SPI TX channel (TEN=1).  DMA starts pre-filling
// TFIFO immediately; the master's first clock edge shifts the
// top byte of the first pre-loaded word out on the data lanes.
static void tx_arm(uint32_t n_words)
{
   dma_oneshot_config(SPI_TX_DMA, (struct dma_buf){dma_tx_buf, n_words},
                      DMA_DIR_TX_FROM_MEM);
   dma_enable(SPI_TX_DMA);
   tx_armed = true;
   spi_tx_enable(SPI_PORT);
}

static void tx_disarm(void)
{
   if (tx_armed) {
      dma_disable(SPI_TX_DMA);
      tx_armed = false;
   }
}

static void spi_rx_flush(void)
{
   while (!(MMR(spi_base + OFF_SPI_STAT) & BIT_SPI_STAT_RFE))
      (void)MMR(spi_base + OFF_SPI_RFIFO);
   MMR(spi_base + OFF_SPI_STAT) =
       BIT_SPI_STAT_ROR | BIT_SPI_STAT_TUR | BIT_SPI_STAT_TC;
}

// Disarm the DMA ring if it was running.  Called before any
// reconfigure that could change RFIFO direction or ring geometry.
static void drain_disarm(void)
{
   if (dma_armed) {
      dma_disable(SPI_RX_DMA);
      dma_armed = false;
   }
}

// Arm the DMA ring for slave RX.  Safe to call repeatedly: reinit
// is idempotent.  After this, data lands in dma_rx_buf continuously
// until the next drain_disarm / reconfigure.
static void drain_arm(void)
{
   if (dma_armed)
      return;
   spi_rx_flush();
   dma_half_words = dma_active_words >> 1U;
   dma_pingpong_rx_config(SPI_RX_DMA, dma_rx_buf, dma_rx_buf + dma_half_words,
                          dma_half_words, dma_desc);
   dma_armed   = true;
   dma_rd_pos  = 0;
   cksum_xor   = 0;
   cksum_count = 0;
}

// Return the number of DMA-written words currently available for
// the consumer to read starting at dma_rx_buf[dma_rd_pos].  Detects
// half-boundary crossings by watching DMA_ADDR_CUR rather than the
// IRQDONE latch: polling DMA_STAT and W1Cing IRQDONE during an
// active transfer appears to stall the channel (data stops landing
// until polling stops).  Reading ADDR_CUR is passive -- HRM warns
// it may be a few cycles stale, but a few bytes of slop in a
// 256 KiB half is irrelevant.
//
// Which half DMA is currently writing is inferred from ADDR_CUR:
//   base .. base+half*4 -> DMA in half 0, half 1 was just drained.
//   base+half*4 .. end  -> DMA in half 1, half 0 was just drained.
// If DMA's current half != the half the consumer is about to read,
// that means the consumer's half has been fully written and is
// safe to drain.
static uint32_t drain_poll_new_words(void)
{
   if (!dma_armed)
      return 0;
   uint32_t hw       = dma_half_words;
   uint32_t half_off = dma_rd_pos;
   uint32_t cpu_half = 0U;
   if (half_off >= hw) {
      half_off -= hw;
      cpu_half = 1U;
   }
   uint32_t addr    = dma_addr_cur(SPI_RX_DMA);
   uint32_t buf_lo  = (uint32_t)(dma_rx_buf);
   uint32_t buf_end = buf_lo + (dma_active_words * BYTES_PER_WORD);
   // DMA_ADDR_CUR is not guaranteed valid until the first data
   // transfer completes (HRM 27-16).  Before then the register
   // can read 0 or any prior value; treat any out-of-range read
   // as "channel not yet running, nothing safe to drain".  This
   // is exactly the window the opening ping-pong race hit: the
   // consumer's half-compare pointed DMA to the opposite half,
   // drained zeros, advanced rd_pos past the real data that then
   // landed, and everything after looked empty.
   if (addr < buf_lo || addr >= buf_end)
      return 0;
   uint32_t byte_off = addr - buf_lo;
   uint32_t dma_half = (byte_off >= (hw * BYTES_PER_WORD)) ? 1U : 0U;
   if (cpu_half == dma_half) {
      // DMA still writing the half the consumer wants to read.
      // Only safe to consume up to what DMA has written in the
      // current half; but the consumer's position is either at the
      // start (no IRQDONE yet for this half) or mid-drain (already
      // saw IRQDONE and some words).  Return 0 -- nothing new is
      // guaranteed complete.
      return 0;
   }
   // DMA has moved into the other half, so cpu_half is fully
   // written from rd_pos to the half end.
   return hw - half_off;
}

// ---------- Parsing helpers ----------
static bool is_space(char c)
{
   return c == ' ' || c == '\t';
}

static const char *skip_ws(const char *s)
{
   while (is_space(*s))
      s++;
   return s;
}

static const char *parse_u32_base(const char *s, uint32_t *out,
                                  unsigned default_base)
{
   s             = skip_ws(s);
   uint32_t v    = 0;
   bool got      = false;
   unsigned base = default_base;
   if (s[0] == '0' && (s[1] == 'x' || s[1] == 'X')) {
      base = BASE_HEX;
      s += 2;
   }
   for (;;) {
      char c     = *s;
      unsigned d = 0;
      if (c >= '0' && c <= '9')
         d = (unsigned)(c - '0');
      else if (base == BASE_HEX && c >= 'a' && c <= 'f')
         d = BASE_DEC + (unsigned)(c - 'a');
      else if (base == BASE_HEX && c >= 'A' && c <= 'F')
         d = BASE_DEC + (unsigned)(c - 'A');
      else
         break;
      v   = v * base + d;
      got = true;
      s++;
   }
   if (got)
      *out = v;
   return s;
}

static const char *parse_u32(const char *s, uint32_t *out)
{
   return parse_u32_base(s, out, BASE_DEC);
}

// ---------- Interactive line reader: echo + backspace + 1-deep history
// ----------
#define CTRL_BS_LEFT   '\b'
#define CTRL_DEL       0x7F
#define CTRL_ESC       0x1B
#define ASCII_PRINT_LO 0x20 // space, first printable
#define ASCII_PRINT_HI 0x7E // tilde, last printable

static char last_line[CMD_BUF_SIZE] = {0};

// Redraw the visible line: CR, erase, re-emit buf.  ANSI "\r\033[K"
// clears from cursor to end of line after carriage return.
static void redraw(const char *buf, uint32_t n)
{
   diag_puts("\r\033[K> ");
   for (uint32_t i = 0; i < n; i++)
      uart_putc(buf[i]);
}

// Forward decl: defined below alongside the DMA drain logic.
static void drain_consume_into_cksum(void);

// Block for the next UART byte.  While idling, piggy-back a DMA
// ring sample so bursts longer than the ring size don't lose data
// between `i` calls.  Gate on `auto_consume` so the feature can
// be disabled for tests that need to observe uncounted ring state.
// How many uart-try polls between drain samples.  Hitting
// DMA_ADDR_CUR at full core speed (~100s of kHz) appears to
// starve the SPI->DMA peripheral-request path on this silicon:
// data stops landing until the polling stops.  One sample per
// AUTO_DRAIN_INTERVAL uart polls is plenty -- each half takes
// ~77 ms to fill at 10 MHz quad, so even 100 Hz drain rate has
// 3 orders of magnitude of margin.
#define AUTO_DRAIN_INTERVAL 1024U

static int uart_getc_block(void)
{
   static uint32_t drain_tick = 0;
   for (;;) {
      int c = uart_try_getc();
      if (c >= 0)
         return c;
      if (auto_consume) {
         drain_tick++;
         if (drain_tick >= AUTO_DRAIN_INTERVAL) {
            drain_tick = 0;
            drain_consume_into_cksum();
         }
      }
   }
}

// Handle an ESC byte just read: consume the two-byte CSI tail, and
// if it is "[ A" (up-arrow) and we have a previous line, copy it
// into buf and redraw.  Returns the new line length.
static uint32_t handle_escape(char *buf, uint32_t cap, uint32_t n)
{
   int c1 = uart_getc_block();
   int c2 = uart_getc_block();
   if (c1 != '[' || c2 != 'A' || last_line[0] == '\0')
      return n;
   uint32_t i = 0;
   while (last_line[i] != '\0' && i + 1U < cap) {
      buf[i] = last_line[i];
      i++;
   }
   redraw(buf, i);
   return i;
}

static void uart_read_line(char *buf, uint32_t cap)
{
   uint32_t n = 0;
   diag_puts("> ");
   for (;;) {
      int c = uart_getc_block();
      if (c == '\r' || c == '\n') {
         diag_puts("\r\n");
         break;
      }
      if (c == CTRL_BS_LEFT || c == CTRL_DEL) {
         if (n > 0) {
            n--;
            diag_puts("\b \b");
         }
         continue;
      }
      if (c == CTRL_ESC) {
         n = handle_escape(buf, cap, n);
         continue;
      }
      if (c < ASCII_PRINT_LO || c > ASCII_PRINT_HI)
         continue; // drop non-printable
      if (n + 1U < cap) {
         buf[n++] = (char)c;
         uart_putc((char)c);
      }
   }
   buf[n] = '\0';
   if (n > 0) {
      for (uint32_t i = 0; i < n && i + 1U < sizeof(last_line); i++)
         last_line[i] = buf[i];
      last_line[n < sizeof(last_line) ? n : sizeof(last_line) - 1] = '\0';
   }
}

// ---------- Hex-dump op ----------
// Walk the already-running DMA ring (or poll RFIFO) and print each
// 32-bit word as "w<idx>=0x<hex>".  If `count_bytes == 0` stream
// forever; any UART byte (next command) stops the stream.
// The DMA ring accumulates data in the background between commands
// -- starting `h` picks up whatever is already pending.
static void op_hex_dump(uint32_t count_bytes, bool forever)
{
   uint32_t words = count_bytes / BYTES_PER_WORD;
   if (forever)
      diag_puts("stream on (any key to stop)\r\n");
   else {
      diag_puts("dump N=0x");
      diag_hex32(count_bytes);
      diag_puts("\r\n");
   }

   uint32_t idx = 0;
   for (;;) {
      if (!forever && idx >= words)
         break;
      if (forever && uart_try_getc() >= 0)
         break;
      uint32_t nw = drain_poll_new_words();
      if (!forever && idx + nw > words)
         nw = words - idx;
      for (uint32_t k = 0; k < nw; k++) {
         uint32_t w = dma_rx_buf[dma_rd_pos];
         diag_puts("w");
         diag_hex32(idx);
         diag_puts("=0x");
         diag_hex32(w);
         diag_puts("\r\n");
         cksum_xor ^= w;
         cksum_count++;
         dma_rd_pos++;
         if (dma_rd_pos == dma_active_words)
            dma_rd_pos = 0;
         idx++;
      }
   }
}

// ---------- Command handlers ----------
static void cmd_mode(const char *rest)
{
   enum spi_miom m = SPI_MIO_SINGLE;
   switch (rest[0]) {
      case '1': m = SPI_MIO_SINGLE; break;
      case '2': m = SPI_MIO_DUAL; break;
      case '4': m = SPI_MIO_QUAD; break;
      default: diag_puts("ERR bad mode (use m1 / m2 / m4)\r\n"); return;
   }
   spi_reconfigure(m);
   diag_puts("mode=");
   diag_puts(miom_name(current_miom));
   diag_puts("\r\n");
}

static void cmd_auto(const char *rest)
{
   rest = skip_ws(rest);
   if (rest[0] == '0')
      auto_consume = false;
   else if (rest[0] == '1')
      auto_consume = true;
   else {
      diag_puts("ERR bad auto (use a 0 or a 1)\r\n");
      return;
   }
   diag_puts(auto_consume ? "auto=on\r\n" : "auto=off\r\n");
}

static void cmd_bufsize(const char *rest)
{
   uint32_t bytes = 0;
   (void)parse_u32(rest, &bytes);
   // Ping-pong splits the ring into two equal halves, so bytes must
   // be a multiple of 2*BYTES_PER_WORD (two 32-bit words).
   if (bytes < PP_MIN_BYTES || (bytes & (PP_MIN_BYTES - 1U)) != 0U) {
      diag_puts("ERR size must be >=8 and a multiple of 8\r\n");
      return;
   }
   uint32_t w = bytes / BYTES_PER_WORD;
   if (w > DMA_BUF_WORDS) {
      diag_puts("ERR exceeds buffer max (524288 B)\r\n");
      return;
   }
   drain_disarm();
   dma_active_words = w;
   drain_arm();
   diag_puts("bufsize=0x");
   diag_hex32(w * BYTES_PER_WORD);
   diag_puts(" B\r\n");
}

// Sample the ring and fold any words that arrived since the last
// drain into the running checksum.  Silent.  Used by `i` to make
// the reported level reflect everything the DMA has landed.
static void drain_consume_into_cksum(void)
{
   if (!dma_armed)
      return;
   // Loop until no more complete halves are pending.  IRQDONE is a
   // single-bit latch; each pass through here clears it (via
   // drain_poll_new_words -> dma_wrap_check) and drains one half,
   // then the next call picks up the following half if DMA has
   // already finished it.  Without the loop, `i` only catches the
   // latest half and loses any earlier halves that piled up while
   // auto_consume was off.
   for (;;) {
      uint32_t nw = drain_poll_new_words();
      if (nw == 0U)
         break;
      for (uint32_t k = 0; k < nw; k++) {
         cksum_xor ^= dma_rx_buf[dma_rd_pos];
         cksum_count++;
         dma_rd_pos++;
         if (dma_rd_pos == dma_active_words)
            dma_rd_pos = 0;
      }
   }
   // Tail drain: pick up any words DMA has written into the current
   // half that haven't crossed the half boundary yet.  Needed for
   // short-payload tests (e.g. first-byte scans) where each CS frame
   // is much smaller than a 256 KiB half so no IRQDONE ever fires.
   // Safe only when DMA is not about to lap CPU -- the short-frame
   // use-case has DMA idle between frames, so no race.
   uint32_t addr    = dma_addr_cur(SPI_RX_DMA);
   uint32_t buf_lo  = (uint32_t)(dma_rx_buf);
   uint32_t buf_end = buf_lo + (dma_active_words * BYTES_PER_WORD);
   if (addr >= buf_lo && addr < buf_end) {
      // >>2 matches /BYTES_PER_WORD; avoids a libc-less __divrem_u32.
      uint32_t dma_pos_words = (addr - buf_lo) >> 2U;
      while (dma_rd_pos != dma_pos_words) {
         cksum_xor ^= dma_rx_buf[dma_rd_pos];
         cksum_count++;
         dma_rd_pos++;
         if (dma_rd_pos == dma_active_words)
            dma_rd_pos = 0;
      }
   }
}

// i: drain pending ring contents into the checksum, print the
// final level + sum, then zero the counters so the next burst
// starts clean.
static void cmd_info(const char *rest)
{
   (void)rest;
   drain_consume_into_cksum();
   uint32_t stat = MMR(spi_base + OFF_SPI_STAT);
   diag_puts("stat=0x");
   diag_hex32(stat);
   diag_puts(" level=0x");
   diag_hex32(cksum_count * BYTES_PER_WORD);
   diag_puts("/0x");
   diag_hex32(dma_active_words * BYTES_PER_WORD);
   diag_puts(" B  sum=0x");
   diag_hex32(cksum_xor);
   // Raw DMA channel state for ping-pong bring-up.  dma_stat's top
   // half holds IRQERR/ERRC (non-zero = rejected config), the low
   // bits hold RUN/IRQDONE.  addr_cur/xcnt_cur let me see whether
   // the channel is transferring at all.
   diag_puts(" dma_stat=0x");
   diag_hex32(dma_stat_raw(SPI_RX_DMA));
   diag_puts(" addr=0x");
   diag_hex32(dma_addr_cur(SPI_RX_DMA));
   diag_puts(" xcnt=0x");
   diag_hex32(dma_xcnt_cur(SPI_RX_DMA));
   diag_puts("\r\n");
   cksum_xor   = 0;
   cksum_count = 0;
}

// p <seed> <bytes>: stage a PRBS stream in dma_tx_buf and start
// slave TX DMA.  The host poller's q.read(n) / q.read_verify_prbs
// can then pull the bytes via MISO (single), D0..D1 (dual), or
// D0..D3 (quad).  <bytes> must be a multiple of 4 and no larger
// than the TX buffer (DMA_TX_WORDS * BYTES_PER_WORD).  FLOW=STOP,
// so TX stops after exactly <bytes> bytes -- the master can clock
// more, but the slave shifter will underrun on subsequent words
// (TUR latches) and pin state is undefined after that.
static void cmd_prbs_tx(const char *rest)
{
   uint32_t seed  = 0;
   uint32_t bytes = 0;
   rest           = parse_u32(rest, &seed);
   (void)parse_u32(rest, &bytes);
   if (bytes == 0U || (bytes & (BYTES_PER_WORD - 1U)) != 0U) {
      diag_puts("ERR bytes must be >0 and a multiple of 4\r\n");
      return;
   }
   if ((bytes >> 2U) > DMA_TX_WORDS) {
      diag_puts("ERR bytes exceeds tx buffer (1 MiB max by default)\r\n");
      return;
   }
   uint32_t n_words = bytes >> 2U;

   // Fill dma_tx_buf with PRBS bytes, packed big-endian into 32-bit
   // words so byte 0 lands in the MSB of word 0.  SPI slave shifter
   // is MSB-first (LSBF=0), so the first bit on the wire is bit 7
   // of byte 0 == PRBS byte 0 bit 7, matching the host generator.
   prbs_init(seed);
   for (uint32_t w = 0; w < n_words; w++) {
      uint32_t b0   = (uint32_t)prbs_next_byte();
      uint32_t b1   = (uint32_t)prbs_next_byte();
      uint32_t b2   = (uint32_t)prbs_next_byte();
      uint32_t b3   = (uint32_t)prbs_next_byte();
      dma_tx_buf[w] = (b0 << (3U * BITS_PER_BYTE)) |
                      (b1 << (2U * BITS_PER_BYTE)) | (b2 << BITS_PER_BYTE) | b3;
   }

   spi_reconfigure_tx(current_miom);
   tx_arm(n_words);

   diag_puts("tx ready seed=0x");
   diag_hex32(seed);
   diag_puts(" bytes=0x");
   diag_hex32(bytes);
   diag_puts(" mode=");
   diag_puts(miom_name(current_miom));
   diag_puts(" ctl=0x");
   diag_hex32(MMR(spi_base + OFF_SPI_CTL));
   diag_puts(" txc=0x");
   diag_hex32(MMR(spi_base + OFF_SPI_TXCTL));
   diag_puts(" fer=0x");
   diag_hex32(MMR(REG_PORTA_FER));
   diag_puts(" dma_stat=0x");
   diag_hex32(dma_stat_raw(SPI_TX_DMA));
   diag_puts(" xcnt=0x");
   diag_hex32(dma_xcnt_cur(SPI_TX_DMA));
   diag_puts("\r\n");
}

static void cmd_hex_dump(const char *rest)
{
   rest           = skip_ws(rest);
   uint32_t count = 0;
   bool forever   = (rest[0] == '\0');
   if (!forever) {
      (void)parse_u32(rest, &count);
      if (count == 0U || (count & 3U) != 0U) {
         diag_puts("ERR count must be >0 and a multiple of 4\r\n");
         return;
      }
   }
   op_hex_dump(count, forever);
}

static void print_help(void)
{
   diag_puts("commands (slave-only):\r\n"
             "  help | ?         print this help + state\r\n"
             "  m1 | m2 | m4     SPI lane width (default m1)\r\n"
             "  b <bytes>        DMA buffer size, <= 524288 B\r\n"
             "  a 0 | a 1        auto-consume off/on (default on);\r\n"
             "                   on = CPU drains ring continuously so\r\n"
             "                   bursts above ring size don't lose words\r\n"
             "  i                drain buffer, print level + running\r\n"
             "                   checksum, reset checksum to zero\r\n"
             "  h                stream received words forever,\r\n"
             "                   stop with any key\r\n"
             "  h <n>            dump exactly <n> bytes (multiple of 4)\r\n"
             "  p <seed> <n>     slave TX: stage PRBS(seed) of <n>\r\n"
             "                   bytes and start DMA transmit (FLOW=STOP)\r\n"
             "editing: backspace deletes, up-arrow recalls last line.\r\n");
   diag_puts("state: mode=");
   diag_puts(miom_name(current_miom));
   diag_puts(" bufsize=0x");
   diag_hex32(dma_active_words * BYTES_PER_WORD);
   diag_puts("\r\n");
}

static bool line_is(const char *s, const char *word)
{
   while (*word) {
      if (*s++ != *word++)
         return false;
   }
   return *s == '\0' || *s == ' ' || *s == '\t';
}

static void handle_command(const char *line)
{
   line   = skip_ws(line);
   char c = line[0];

   if (c == '\0')
      return;
   if (c == '?' || line_is(line, "help")) {
      print_help();
      return;
   }

   switch (c) {
      case 'm': cmd_mode(line + 1); break;
      case 'b': cmd_bufsize(line + 1); break;
      case 'a': cmd_auto(line + 1); break;
      case 'i': cmd_info(line + 1); break;
      case 'h': cmd_hex_dump(line + 1); break;
      case 'p': cmd_prbs_tx(line + 1); break;
      case 'q': {
         // DIAG: polled-TX bring-up.  No DMA, no PRBS.  Switch to
         // slave TX at current lane width, shove one fixed pattern
         // into TFIFO a few times, enable TEN, report state.
         // Polled-TX bring-up pattern.  Four fixed words direct-
         // written to TFIFO; DSP shifts them out MISO on next CS.
#define DIAG_TFIFO_W0 0xAA55AA55U
#define DIAG_TFIFO_W1 0xDEADBEEFU
#define DIAG_TFIFO_W2 0xCAFEBABEU
#define DIAG_TFIFO_W3 0x12345678U
         // HRM 15-30 slave programming order: write CTL/RXCTL/
         // TXCTL first (with EN=0 during setup), fill TFIFO, then
         // enable CTL.EN last.  Currently spi_reconfigure_tx leaves
         // CTL.EN=1, so drop it before the TFIFO prime and raise
         // again after.
         spi_reconfigure_tx(current_miom);
         MMR(spi_base + OFF_SPI_CTL) &= ~BIT_SPI_CTL_EN;
         MMR(spi_base + OFF_SPI_TXCTL) |= BIT_SPI_TXCTL_TEN;
         MMR(spi_base + OFF_SPI_RXCTL) |= BIT_SPI_RXCTL_REN;
         MMR(spi_base + OFF_SPI_TFIFO) = DIAG_TFIFO_W0;
         MMR(spi_base + OFF_SPI_TFIFO) = DIAG_TFIFO_W1;
         MMR(spi_base + OFF_SPI_TFIFO) = DIAG_TFIFO_W2;
         MMR(spi_base + OFF_SPI_TFIFO) = DIAG_TFIFO_W3;
         MMR(spi_base + OFF_SPI_CTL) |= BIT_SPI_CTL_EN;
         diag_puts("polled tx loaded ctl=0x");
         diag_hex32(MMR(spi_base + OFF_SPI_CTL));
         diag_puts(" txc=0x");
         diag_hex32(MMR(spi_base + OFF_SPI_TXCTL));
         diag_puts(" stat=0x");
         diag_hex32(MMR(spi_base + OFF_SPI_STAT));
         diag_puts("\r\n");
         break;
      }
      case 'x':
         diag_puts("tx_stat spi_stat=0x");
         diag_hex32(MMR(spi_base + OFF_SPI_STAT));
         diag_puts(" txc=0x");
         diag_hex32(MMR(spi_base + OFF_SPI_TXCTL));
         diag_puts(" dma_stat=0x");
         diag_hex32(dma_stat_raw(SPI_TX_DMA));
         diag_puts(" xcnt=0x");
         diag_hex32(dma_xcnt_cur(SPI_TX_DMA));
         diag_puts(" addr=0x");
         diag_hex32(dma_addr_cur(SPI_TX_DMA));
         diag_puts("\r\n");
         break;
      default:
         diag_puts("ERR unknown cmd\r\n");
         print_help();
         break;
   }
}

int main(void)
{
   static const struct clocks_cfg clk = BOARD_CLOCKS_CFG;
   clocks_init(&clk);
   uart_init(BOARD_BAUD_DIV);
   timer_init();
   delay_ms(STARTUP_MS);

   diag_puts("\r\nqspi shell starting\r\n");

   spi_base = REG_SPI0_BASE + ((uint32_t)SPI_PORT * SPI_STRIDE);
   spi_reconfigure(SPI_MIO_SINGLE);

   diag_puts("ready. type `help` for commands.\r\n");

   static char line[CMD_BUF_SIZE];
   for (;;) {
      uart_read_line(line, CMD_BUF_SIZE);
      handle_command(line);
   }
}
