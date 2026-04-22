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

## 0.5. SPY sanity (SPI RX path, no UART commands)

Before Step 1, confirm the SPI slave actually latches bytes:

- Firmware boots into a SPY loop that auto-dumps every SPI
  RX word to UART without needing a command. Send a qspi job
  with only `delay_us(100000)` + `write_prbs(0xC0FFEE, 64)` +
  `delay_us(500000)` -- no uart_tx ops at all.
- Expect 16 `SPY w=<hex>` lines (one per 32-bit word), ROR=0,
  and the PRBS stream decodes against the reference.

**Status 2026-04-21: PASS.** Slave-role SPI2 now receives
the full 16-word PRBS burst from the FT4222-as-master
host. Verified on hardware via
`python3 ../../test_serv/submit.py --wait 60 --runtime 8
--qspi spy.qspi main.ldr` with `SPY_ENABLE=1`: 16
`SPY w=<hex>` lines (0x709a0e4a, 0xab40d513, ...,
0x24437618) exactly match the bit-stream produced by
`prbs_xorshift32(0x00C0FFEE, 64)` in
`test_serv/poller.py`, and `ror=0` in the trailing status
line.

Root cause was three simultaneous bugs; any one of them
alone produced "RFE stays latched" and masked the
others:

1. **SPI_SLVSEL inherited from master SELFTEST.** HRM
   15-68 (Table 15-31): `SPI_SLVSEL.SSE1` "enables the
   related SPI_SEL[n] pin for output. If disabled, the SPI
   three-states the related SPI_SEL[n] pin." A prior
   master-mode SELFTEST left SSE1=1, so the peripheral
   drove PA5 high from inside the DSP while acting as a
   slave; the external FT4222 CS0 could not pull the line
   low through the DSP's own driver, and the slave never
   saw an SS falling edge. Fix: write SLVSEL
   unconditionally on both transitions; slave value =
   0x0000FE00, matching CCES's `DEFAULT_SPISLVSEL_OUTPUT`
   in `adi_spi_data_2156x.c`.

2. **PORTA_FER on slave input pins was load-bearing
   incorrect.** HRM 12-41: "The function enable bits
   **impact output control only**. Regardless of the
   setting of the function enable bits, both GPIO and
   peripherals can still sense the pin input."
   Empirically on the 21569, forcing FER=1 on a slave's
   input pins (MOSI, SCLK, SS, unused D2/D3) prevents the
   receive shift register from promoting completed words
   into RFIFO -- RFE stays latched for the full burst,
   indistinguishable from "no clocks received at all".
   Setting FER=0 on those pins (peripheral still reads the
   pad per the HRM quote) lets every word land. Fix:
   `pinmux_spi2` enables FER only on output pins -- in
   master role all of PA0..PA5, in slave role only PA0
   (MISO output).

3. **SPI_CTL.CPHA sampled the wrong clock edge.**
   `test_serv/poller.py :: _cpol_cpha` with flags=0 maps
   to pyft4222's `Cpha.CLK_TRAILING`, which in standard
   SPI terms is CPHA=1 (data latched on the second clock
   edge). At CPHA=0 the few bytes that did land were all
   right-shifted by one bit position. Fix: `.cpha =
   master ? 0U : 1U` in `qspi/main.c :: spi_reconfigure`.

Supporting improvements (principled per HRM / CCES
reference driver, but not individually sufficient):

- `OSPI0_CTL.EN` cleared before SPI2 bring-up (boot ROM
  may leave OSPI0 enabled and it contends with SPI2 for
  the shared PA0..PA5 pin group).
- `REG_SCB5_REMAP` cleared -- note this is a memory-map
  remap only, not pin routing (see ADSP-SC592_typedefs.h
  :: SCB5_SPI2_OSPI_REMAP enum; 21569 variant is 1-bit,
  0=SPI2).
- `SPI_TXCTL.TTI` / `SPI_RXCTL.RTI` cleared in slave mode.
  Per HRM 15-66 both fields are "valid only when the SPI
  is a master"; the CCES reference driver
  `adi_spi_2156x.c :: EnableSPIChannel` clears both in
  the slave branch.
- `pinmux_spi2` for PA5: mux=1 (alt "b" = SPI2_SEL1b
  output when master, SPI2_SSb input when slave). PORT_MUX
  is 2 bits / pin (HRM 19-28), so any earlier claim that
  SPI2_SSb lives at mux=3 was wrong -- that selects
  SMC0_D05 (static memory controller data line).

Bench diagnostic scaffold (SPY_ENABLE in `qspi/main.c`)
is left in the tree, gated off by default. Set to 1 to
re-run Step 0.5 before shipping Steps 1-4. With
SPY_ENABLE=0 the command loop starts immediately after
boot and Steps 5-10 proceed as before.

## 1. Slave, single-lane, polled (command baseline)

Once SPY proves the SPI slave captures clean words:

- Send `M1\n` then `P c0ffee 64\n` over UART.
- Host queues a matching `write_prbs(0xC0FFEE, 64)` on SPI2.
- Expect: `PRBS mode=x1 seed=0x00c0ffee N=64 OK ticks=.. ror=0`.
- If this fails, nothing else will; check pinmux, clocks,
  SPI_CTL, RX FIFO polling before anything else.

**Status 2026-04-21: PASS.** Hardware capture:
`PRBS mode=x1 seed=0x00c0ffee N=64 OK ticks=9296213 ror=0`
after `M1\n` then `P c0ffee 64\n` followed by the host
`write_prbs(0xC0FFEE, 64)`.

Root cause of initial bring-up failure was one missing step
in `pinmux_spi2`: **PORTA_INEN was never asserted for the
SPI2 pin group.** HRM 12-49: PORTA_INEN[n]=1 enables the pad's
input buffer; with INEN=0 the peripheral input connected to
that pin reads 0 regardless of the external signal. Step 0.5
happened to set `PORTA_INEN_SET |= PA0..PA5` as a bring-up
aid inside the SPY scaffold, which is why SPY captured the
full burst; once SPY was disabled the slave never saw SCLK /
MOSI / SS edges and RFE stayed latched with SPI_STAT =
0x00440001 (RFE=1, TFIFO empty, SPIF idle) for the entire
wait window. FER only controls the output driver (HRM 12-41:
"the function enable bits impact output control only"), so
FER and INEN are orthogonal -- both must be programmed for a
peripheral that sources or sinks data through the pad. Fix:
`pinmux_spi2` now writes `PORTA_INEN_SET = PA0..PA5` on every
call, which is correct for master role too (MISO input, MODF
sense on SEL1).

## 2. Slave, single-lane, DMA

- Same as above but command `D c0ffee 64\n`.
- Expect `PRBSDMA ... OK`. Verifies SPI2_RX DMA channel (DMA27)
  and the new `dma_oneshot_config` path in common/dma.c.

**Status 2026-04-21: PASS.** Hardware capture:
`PRBSDMA mode=x1 seed=0x00c0ffee N=64 OK ticks=9313977 ror=0`
after `M1\n` then `D c0ffee 64\n` followed by the host
`write_prbs(0xC0FFEE, 64)`. Passed on first try against the
fixes already landed in Step 0.5 / Step 1; the `dma_oneshot_config`
path in `common/dma.c` and `spi_rx_dma_enable` in `common/spi.c`
deliver 16 32-bit words (64 B) from SPI2_RX -> DMA27 -> `dma_rx_buf`
cleanly, with zero ROR. Regression re-run of step1.qspi and
step5.qspi both still PASS (`PRBS ... OK ticks=9300723 ror=0`,
`TX0 mode=x1 N=64 ticks=151 tur=0`).

## 3. Slave, lane-width sweep

- `M2` then repeat steps 1 and 2; then `M4` and repeat.
- Watch for ROR once bit rate climbs. FT4222 max ~30 Mbit/s in
  quad @ DIV_8; DMA must keep up.

**Status 2026-04-21: PASS.** Combined job `step3.qspi` exercises
all four (lane-width x polled/DMA) variants in one DSP session
(`M2 -> P -> D -> M4 -> P -> D`) with a host-side `reinit` from
DUAL to QUAD between the M2 and M4 blocks. Hardware capture,
single submit:

    PRBS    mode=x2 seed=0x00c0ffee N=64 OK ticks=9302197 ror=0
    PRBSDMA mode=x2 seed=0x00c0ffee N=64 OK ticks=9295393 ror=0
    PRBS    mode=x4 seed=0x00c0ffee N=64 OK ticks=9282474 ror=0
    PRBSDMA mode=x4 seed=0x00c0ffee N=64 OK ticks=9315564 ror=0

Two changes were required to make dual/quad slave RX work:

1. **Host-side: the stock `write_prbs` (tag 0x07) uses
   `spiMaster_SingleWrite`, which in FT4222 only drives MOSI
   (D0) regardless of the init mode.** It cannot exercise the
   DSP's dual/quad slave receiver even when the FT4222 master
   is initialised with `MODE_DUAL` / `MODE_QUAD`. Added
   `make_qspi.write_prbs_multi(seed, n)` which wraps
   `mixed_xfer(b"", prbs, 0)` -- the payload lands in the
   MultiReadWrite multi-IO write phase and is actually
   spread across D0/D1 (dual) or D0..D3 (quad).

2. **Firmware-side: `pinmux_spi2` kept PA0 (D1 / MISO) FER=1
   in every slave role, which is correct for single-lane
   slave (where PA0 is the sole driven output) but wrong for
   dual/quad slave receive**, where PA0..PA3 are all inputs
   from the external master; leaving FER=1 on a received
   data lane has the DSP's pad driver fight the host. Fix:
   `pinmux_spi2` now takes a `miom` parameter, and the slave
   branch sets FER=1<<0 only for SPI_MIO_SINGLE. Dual and
   quad slave RX clear FER on all of PA0..PA3 (INEN stays
   set on the whole PA0..PA5 group per Step 1's fix, so the
   peripheral still reads the pads).

Regression after the fix: step1.qspi, step2.qspi, step5.qspi
all still PASS (`PRBS mode=x1 ... OK ticks=9286555 ror=0`,
`PRBSDMA mode=x1 ... OK ticks=9287895 ror=0`,
`TX0 mode=x1 N=64 ticks=151 tur=0`).

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

**Status 2026-04-21: PARTIAL.** DSP master role works for
small bursts. SELFTEST clocks 4 words successfully, and
`Rm 9\nL N` handles N <= 16 bytes (up to TFIFO depth = 4
words), printing `TX0 mode=x1 N=.. ticks=.. tur=0`. N > 16
hangs inside `op_tx_zeros`: the `while (TFF)` wait never
clears at index 4 because SPI stops transmitting after the
first TFIFO-depth batch -- SPI_STAT reads `0x00804000`
(TFF=1 + bit 14 set) and TS stays low, so the peripheral is
idle while the FIFO is full.  Candidates: master needs a
per-burst TTI re-trigger, or the ASSEL/SLVSEL programming
keeps CS asserted and blocks the next burst.  Bring-up
scaffolding in op_tx_zeros now has a 100 ms spin timeout
that prints `TX0 stuck i=... stat=...` so the hang doesn't
take the whole job window.

Steps 6-9 reuse the same refill pattern (op_prbs_tx,
op_prbs_rx_master, op_prbs_xfer) and will hit the same
ceiling until the TFIFO-refill path is fixed.

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
