# qspi hardware verification plan

Bring-up order for the new command protocol on real hardware.
Each step depends on the previous one passing; stop and debug
before moving on.

## Submit recipe

Every step runs as one stapled job so UART + scope span reset
-> boot -> qspi opcodes -> trailing window in a single capture:

    cd adsp2156/qspi
    make -j24
    python3 ../../test_serv/submit.py --wait 30 --runtime 6 \
        --qspi stepN.qspi main.ldr

Build each step's `.qspi` with a tiny Python script that pulls
in `test_serv/examples/make_qspi.py` helpers.

## 0. Prerequisites (must be in place before Step 1)

These are firmware bugs uncovered during bring-up; every later
step assumes they are fixed.

- `common/spi.h :: struct spi_cfg` must use `uint32_t clkdiv`,
  not `uint16_t`. cc21k generates a halfword store that the
  SHARC+ core traps on (FAULT LED latched, C2 ~= 124 in the
  scope CSV) when writing a `uint16_t` field on the stack in
  `spi_reconfigure`.
- `common/uart.c :: uart_try_getc` must W1C the OE/PE/FE/BI
  error bits of `UART0_STAT`. Once any of them latches -- and
  OE latches easily when the DSP is mid-printf -- the UART
  stops asserting DR for subsequent bytes and RX dies
  silently. Order matters: read RBR first when DR is set,
  only clear errors when DR was 0, or the W1C clears DR as
  a side effect.
- `qspi/main.c :: parse_seed_count` must default the seed to
  hex (the doc string says `<seed_hex>`). Bare `c0ffee` would
  otherwise parse as decimal, fail silently, and leave the
  count stuck at 0.
- FAULT LED check: before any step, confirm the `.csv` shows
  `C2 DSP_FAULT duty=0%`. >0% means the core halted and any
  UART/SPI result is meaningless.

## 0.5. SPY sanity (SPI RX path, no UART commands)

Before Step 1, confirm the SPI slave actually latches bytes:

- Firmware boots into a SPY loop that auto-dumps every SPI
  RX word to UART without needing a command. Send a qspi job
  with only `delay_us(100000)` + `write_prbs(0xC0FFEE, 64)` +
  `delay_us(500000)` -- no uart_tx ops at all.
- Expect 16 `SPY w=<hex>` lines (one per 32-bit word), ROR=0,
  and the PRBS stream decodes against the reference.

**Status 2026-04-21: PARTIALLY BLOCKED.** SPI_CTL reads back
cleanly as `0x00000501` (EN | EMISO | SIZE=2) after the
struct-field fix. A boot-time SELFTEST that configures the
module as master and clocks four 32-bit words out of PA4
lands four full-duplex words in the RX FIFO, proving the
peripheral, its clock, its pinmux at PA0..PA5 mux=1 ("b"),
and the SCB5 pin routing are all functional.

What does NOT work: running the module as slave and having
the FT4222-master host clock bytes in. No bytes ever reach
the RX FIFO, RFE stays latched, ROR never fires, across
every combination tried:

- PA0..PA4 alt-function mux=0/1/2/3
- PA5 alt-function mux=0/1/2/3
- SCB5_REMAP = 0 and 1
- CPOL=0 with CPHA=0 and CPHA=1
- PSSE on and off (ignore-SS-pin vs honor-SS-pin)
- SOSI on and off (swap MISO/MOSI role in slave)

Because the master-mode self-test exercises the same pin
group and is fine, and no firmware knob left to turn, the
remaining candidates are all electrical / board-level:

- FT4222's SCK / MOSI / SS lines routed to different DSP
  pins than the SPI2 peripheral uses (USB_QSPI expander
  path may land on OSPI pins only, which SCB5_REMAP would
  decide between at boot time).
- FT4222's master outputs floating / disabled after the
  LDR-boot phase of the unified capture session.

Defer Steps 1-4 (slave-role PRBS) until that electrical
question is answered. Move straight to Steps 5-10, which
drive the bus from the DSP as master -- the SELFTEST
proves that path already works.

## 1. Slave, single-lane, polled (command baseline)

Once SPY proves the SPI slave captures clean words:

- Send `M1\n` then `P c0ffee 64\n` over UART.
- Host queues a matching `write_prbs(0xC0FFEE, 64)` on SPI2.
- Expect: `PRBS mode=x1 seed=0x00c0ffee N=64 OK ticks=.. ror=0`.
- If this fails, nothing else will; check pinmux, clocks,
  SPI_CTL, RX FIFO polling before anything else.

## 2. Slave, single-lane, DMA

- Same as above but command `D c0ffee 64\n`.
- Expect `PRBSDMA ... OK`. Verifies SPI2_RX DMA channel (DMA27)
  and the new `dma_oneshot_config` path in common/dma.c.

## 3. Slave, lane-width sweep

- `M2` then repeat steps 1 and 2; then `M4` and repeat.
- Watch for ROR once bit rate climbs. FT4222 max ~30 Mbit/s in
  quad @ DIV_8; DMA must keep up.

## 4. Slave, long-burst

- `M4`, then `D decaf 1048576\n`. Checks that chunked DMA
  re-arm (16 KiB units) does not drop bytes under sustained
  clocking. If ROR appears, switch to dual-buffer AUTO mode.

## 5. Master bring-up, no PRBS

- Switch host to FT4222-as-slave config.
- `M1\nRm 9\n` -> 9.375 MHz SCK (under the 20 MHz FT4222 slave
  ceiling with plenty of headroom).
- `L 64\n` on DSP (clocks zero bytes). Confirm host sees 64
  zero bytes on MOSI and CS toggles low-then-high.
- If CS does not assert, check BIT_SPI_CTL_ASSEL in
  common/spi.c :: spi_init.

## 6. Master, WR (PRBS transmit)

- `P c0ffee 64\n`. Host verifies incoming PRBS against the
  same seed. Expect `PRBSTX ... tur=0`.

## 7. Master, RD (PRBS receive+verify)

- Host queues PRBS(0xDECAF, 64) on FT4222-slave MISO with
  timeout.
- DSP: `V decaf 64\n`.
- Expect `PRBSRX ... OK`.

## 8. Master, XF (full-duplex PRBS)

- DSP: `X c0ffee 64\n`; host generates PRBS(0xC0FFEE) on MISO
  and verifies MOSI against the same.
- Expect `PRBSXF ... OK`.

## 9. Master rate sweep

- Step clkdiv down: `Rm 9`, `Rm 6`, `Rm 4` (18.75 MHz). Repeat
  step 8 at each rate. Above `Rm 4` violates the 20 MHz FT4222
  slave spec -- do not try it on this hardware.

## 10. Burst-size sweep

- Scripts of 64 / 4096 / 16384 / 65536 byte PRBS ops in master
  role at `Rm 4`. FT4222 slave USB endpoint is 4160 B; the
  memo recommends <= 4 kB bursts with gaps. Confirm behavior
  under and over that threshold; if larger bursts corrupt,
  split them host-side.

## Notes

- UART echoes a `READY role=.. mode=..` line after reset; use
  it as a probe that the firmware is alive.
- `?` prints current state at any time; issue before a new
  burst to confirm the expected role and mode.
- Quad master is not supported against FT4222 (slave is
  single-lane only). Skip any `M4` + `Rm` combination.
- Once a step passes, remove the bring-up scaffolding from
  `qspi/main.c` (SPY loop, heartbeat, `RX[...]>...<` line
  echo, UART STAT dumps) before moving on -- they add noise
  to later captures.
