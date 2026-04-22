// SPDX-License-Identifier: MIT
// main.c --- SPI2 interactive hex-dump shell
// Copyright (c) 2026 Jakob Kastelic

// UART0 is a tiny interactive shell that configures SPI2 and drains
// receive data as raw hex.  No PRBS, no loopback.  Intended for
// bring-up: point FT4222 at SPI2, open a terminal on UART0, type
// commands, watch words.
//
// Commands (see `help` for the live list):
//   help | ?           print command list + current state
//   rs                 role slave (default)
//   rm <clkdiv>        role master at SCLK0 / (clkdiv + 1)
//   m1 | m2 | m4       lane width: 1 (default), 2, 4
//   b <bytes>          DMA buffer size
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
#define SPI_STRIDE     0x1000U
#define STARTUP_MS     500U
#define BYTES_PER_WORD 4U
#define BITS_PER_BYTE  8U
#define BYTE_MASK      0xFFU
#define CMD_BUF_SIZE   128U
#define BASE_DEC       10U
#define BASE_HEX       16U

// DMA receive buffer, in 32-bit words.  FLOW=AUTO ring; the CPU
// chases XCNT_CUR.  Landed in L2 SRAM via seg_l2_bss -- L1 block1
// faults the core at reset past ~32 KB of initialised data.  L2 has
// ~1 MiB total; 512 KiB here leaves headroom for other seg_l2_bss
// data and stays well clear of any future static allocations.
#define DMA_BUF_WORDS (512U * 1024U / BYTES_PER_WORD)

#pragma section("seg_l2_bss", NO_INIT)
static uint32_t dma_rx_buf[DMA_BUF_WORDS];

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
static bool current_master        = false;
static uint32_t current_clkdiv    = 0;
// Active DMA ring size in 32-bit words.  Capped by the static
// dma_rx_buf[] allocation (DMA_BUF_WORDS).  b command adjusts it.
static uint32_t dma_active_words = DMA_BUF_WORDS;
// Running XOR-checksum + word count accumulated since the last
// `i` command (which prints and resets them).  No auto-reset on
// buffer fill -- if the ring wraps faster than the host drains,
// data is lost silently.
static uint32_t cksum_xor   = 0;
static uint32_t cksum_count = 0;
// Always-on DMA ring bookkeeping.  Armed at boot in slave role so
// data received before `h` is already in the ring; `h` and `i`
// walk rd_pos forward, sampling XCNT_CUR to absorb the new words
// the DMA has landed since the previous sample.
static bool dma_armed       = false;
static uint32_t dma_rd_pos  = 0;
static uint32_t dma_prev_xc = 0;
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

static void spi_reconfigure(enum spi_miom miom, bool master, uint32_t clkdiv)
{
   // Board convention: CPOL=0, CPHA=1.  Matches pyft4222
   // Cpha.CLK_TRAILING (what flags=0 selects in poller.py).
   struct spi_cfg cfg = {
       .clkdiv    = clkdiv,
       .size      = SPI_WORD_32,
       .miom      = miom,
       .is_master = master ? 1U : 0U,
       .cpol      = 0,
       .cpha      = 1U,
       .lsb_first = 0,
   };
   drain_disarm();
   pinmux_spi2(master ? 1 : 0, (unsigned)miom);
   spi_init(SPI_PORT, &cfg);
   if (!master)
      spi_rx_dma_enable(SPI_PORT);
   // TEN only when DSP sources data.  In slave dual/quad the TX
   // shifter has no useful data; TEN=1 + underrun drives idle
   // pattern onto MISO/D1 (PA0) and contends with the master.
   if (master)
      spi_tx_enable(SPI_PORT);
   spi_rx_enable(SPI_PORT);
   current_miom   = miom;
   current_master = master;
   current_clkdiv = clkdiv;
   drain_arm();
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
// until the next drain_disarm / reconfigure.  Master role has no
// RX to arm -- silently disarms instead.
static void drain_arm(void)
{
   if (current_master) {
      drain_disarm();
      return;
   }
   if (dma_armed)
      return;
   spi_rx_flush();
   dma_autobuffer_config(SPI_RX_DMA,
                         (struct dma_buf){dma_rx_buf, dma_active_words},
                         DMA_DIR_RX_TO_MEM);
   dma_enable(SPI_RX_DMA);
   // Swallow any stale IRQDONE latched before arming.
   (void)dma_wrap_check(SPI_RX_DMA);
   dma_armed   = true;
   dma_rd_pos  = 0;
   dma_prev_xc = dma_active_words;
   cksum_xor   = 0;
   cksum_count = 0;
}

// Sample XCNT_CUR and return the count of new words the DMA has
// written since the last sample.  Updates dma_prev_xc.  Safe to
// call many times per second; the ring wraps and we track the
// delta modulo dma_active_words.  Caller must consume / print
// those words from dma_rx_buf[dma_rd_pos] onward and advance
// rd_pos by the returned count.
// Sample XCNT_CUR and return the count of new words the DMA has
// written since the last sample.  Uses DMA_STAT.IRQDONE (latched
// on every XCNT_CUR underflow = ring wrap) so multi-wrap intervals
// are counted correctly.  Call at least once per ring-fill-time.
static uint32_t drain_poll_new_words(void)
{
   if (!dma_armed)
      return 0;
   uint32_t cur = dma_xcnt_cur(SPI_RX_DMA);
   uint32_t nw  = (cur <= dma_prev_xc) ? (dma_prev_xc - cur)
                                       : (dma_prev_xc + dma_active_words - cur);
   dma_prev_xc  = cur;
   return nw;
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
static int uart_getc_block(void)
{
   for (;;) {
      int c = uart_try_getc();
      if (c >= 0)
         return c;
      if (auto_consume)
         drain_consume_into_cksum();
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
static void cmd_role(const char *rest)
{
   if (rest[0] == 's') {
      spi_reconfigure(current_miom, false, 0);
      diag_puts("role=slave mode=");
      diag_puts(miom_name(current_miom));
      diag_puts("\r\n");
   } else if (rest[0] == 'm') {
      uint32_t clkdiv = 0;
      (void)parse_u32(rest + 1, &clkdiv);
      spi_reconfigure(current_miom, true, clkdiv);
      diag_puts("role=master clkdiv=0x");
      diag_hex32(clkdiv);
      diag_puts(" mode=");
      diag_puts(miom_name(current_miom));
      diag_puts("\r\n");
   } else {
      diag_puts("ERR bad role (use rs or rm <clkdiv>)\r\n");
   }
}

static void cmd_mode(const char *rest)
{
   enum spi_miom m = SPI_MIO_SINGLE;
   switch (rest[0]) {
      case '1': m = SPI_MIO_SINGLE; break;
      case '2': m = SPI_MIO_DUAL; break;
      case '4': m = SPI_MIO_QUAD; break;
      default: diag_puts("ERR bad mode (use m1 / m2 / m4)\r\n"); return;
   }
   spi_reconfigure(m, current_master, current_clkdiv);
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
   if (bytes == 0U || (bytes & 3U) != 0U) {
      diag_puts("ERR size must be >0 and a multiple of 4\r\n");
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
   uint32_t nw = drain_poll_new_words();
   for (uint32_t k = 0; k < nw; k++) {
      cksum_xor ^= dma_rx_buf[dma_rd_pos];
      cksum_count++;
      dma_rd_pos++;
      if (dma_rd_pos == dma_active_words)
         dma_rd_pos = 0;
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
   diag_puts("\r\n");
   cksum_xor   = 0;
   cksum_count = 0;
}

static void cmd_hex_dump(const char *rest)
{
   if (current_master) {
      diag_puts("ERR h is slave-only\r\n");
      return;
   }
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
   diag_puts("commands:\r\n"
             "  help | ?         print this help + state\r\n"
             "  rs               role slave (default)\r\n"
             "  rm <clkdiv>      role master, dec or 0xhex\r\n"
             "  m1 | m2 | m4     SPI lane width (default m1)\r\n"
             "  t 0 | t 1        drain: 0=polled, 1=DMA (default DMA)\r\n"
             "  b <bytes>        DMA buffer size, <= 524288 B\r\n"
             "  a 0 | a 1        auto-consume off/on (default on);\r\n"
             "                   on = CPU drains ring continuously so\r\n"
             "                   bursts above ring size don't lose words\r\n"
             "  i                drain buffer, print level + running\r\n"
             "                   checksum, reset checksum to zero\r\n"
             "  h                stream received words forever,\r\n"
             "                   stop with any key\r\n"
             "  h <n>            dump exactly <n> bytes (multiple of 4)\r\n"
             "editing: backspace deletes, up-arrow recalls last line.\r\n");
   diag_puts("state: role=");
   diag_puts(current_master ? "master" : "slave");
   diag_puts(" mode=");
   diag_puts(miom_name(current_miom));
   diag_puts(" clkdiv=0x");
   diag_hex32(current_clkdiv);
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
      case 'r': cmd_role(line + 1); break;
      case 'm': cmd_mode(line + 1); break;
      case 'b': cmd_bufsize(line + 1); break;
      case 'a': cmd_auto(line + 1); break;
      case 'i': cmd_info(line + 1); break;
      case 'h': cmd_hex_dump(line + 1); break;
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
   spi_reconfigure(SPI_MIO_SINGLE, false, 0);

   diag_puts("ready. type `help` for commands.\r\n");

   static char line[CMD_BUF_SIZE];
   for (;;) {
      uart_read_line(line, CMD_BUF_SIZE);
      handle_command(line);
   }
}
