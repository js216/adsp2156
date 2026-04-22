# qspi hardware verification plan

Bring-up order for the new command protocol on real hardware.
Each step depends on the previous one passing; stop and debug
before moving on.

Check documentation in doc/ for any hardware questions.

## FT4222 multi-IO first-byte hazard (MUST READ)

Empirically confirmed on this hardware, AD eval board + FT4222
connected as SPI master, DSP SPI2 slave:

**The first byte clocked out by the FT4222 in a dual-IO or
quad-IO `SPIMaster_MultiReadWrite` CS frame can corrupt the
DSP's D1-lane sampling for the entire rest of that CS frame.**

Symptom: the DSP receives word 0 correctly, but from word 1
onward the D1 lane (PA0 / MISO) reads as stuck-at-0 -- every
expected `1` bit in a D1 position is sampled as `0`. D0, D2,
D3 remain clean.

Trigger: content-specific. Specific byte values at the start
of the CS frame (e.g. 0x0d4416c4, s123[0:4]) poison the
receiver for the duration; most byte values (e.g. 0x00000000,
0xFFFFFFFF, 0xc8ad14e5, random seeds that don't hit the
trigger) leave it clean. An exhaustive 2^32 sweep is infeasible
(~163 years at 1 s/test), so the safe rule is simpler than
enumerating bad patterns.

Rule: **every dual-IO / quad-IO CS frame must start with a
zero word** (four bytes of `0x00`). That is:

    dev.spiMaster_MultiReadWrite(b"", b"\x00\x00\x00\x00" + payload, 0)

Verified clean on 64 KiB + 4 B streams via chunked TLVs, each
chunk prefixed with a zero word, all bytes accounted for and
checksum-matched.

Why this works: per AN_329 section 3.4.6 `FT4222_SPISlave_Write`:

> "For some reasons, support lib will append a dummy byte
>  (0x00) at the first byte automatically. This additional
>  byte exists at all the three transfer methods."

FTDI documents the equivalent auto-prepend for the slave-write
path but not for `MultiReadWrite`. Our findings match: same
underlying chip hazard, not worked around in the master multi-IO
path. Prepending the zero word replicates the fix manually.

Implementation choice (see discussion in session log): keep
the prefix in the test-case generators, not in `poller.py`.
Reasons: poller stays a dumb byte forwarder; single-IO tests
(tag 0x07 `write_prbs`, tag 0x01 `write`) don't need the
prefix; pattern-probing tests (w0 sweeps) deliberately send
non-zero first words; DSP firmware needs no changes. Helper
in `make_qspi.py`:

    def mixed_xfer_safe(single_buf, multi_w_buf, nread):
        return mixed_xfer(single_buf, b"\x00\x00\x00\x00"
                          + multi_w_buf, nread)

Checksum math is unaffected: `running_xor(b"\x00\x00\x00\x00"
+ data) == running_xor(data)`, so existing test assertions on
host-side computed sums still match the DSP's XOR accumulator.

Per-call cap on `MultiReadWrite`: libft4222 doc says 65535 B
max for `multiWriteBytes`, but empirical `FAILED_TO_WRITE_DEVICE`
at 65532 B suggests the real cap is lower. Use TLV chunks
<= 65528 B including the 4 B zero prefix (= 65524 B of user
payload per chunk) to stay safely under.

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
