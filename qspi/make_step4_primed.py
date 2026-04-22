# SPDX-License-Identifier: GPL-3.0
# make_step4_primed.py --- Workaround probe: polled P primes before DMA D
# Copyright (c) 2026 Jakob Kastelic

# Quad-slave DMA corrupts from byte 20 when invoked directly after a
# mode switch, but running a polled PRBS receive first clears the
# fault.  This probe runs P 16 B (= one word that the peripheral
# internally handles in the polled code path) ahead of the 32 KiB
# DMA burst and checks whether the DMA survives.

import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__),
                                "../../test_serv/examples"))
import make_qspi as q

PRIME_SEED  = 0x000DECAF
PRIME_BYTES = 64
DMA_SEED    = 0x000DECAF
DMA_BYTES   = 64

# Host must continue the PRBS stream into the DMA burst without
# resetting the seed, since the DSP does not reseed between P and D.
# Instead we run two independent seeds: prime with one seed, drain,
# then start D with a different seed.
PRIME = 0x11112222
DMA_S = 0x000DECAF

buf = bytearray()
buf += q.header(clk_div=q.CLK_DIV_8, mode=q.MODE_QUAD, flags=0)

buf += q.uart_tx(b"M4\n")
buf += q.delay_us(500000)

# ---- Prime: polled P, 16 B quad ----
buf += q.uart_tx(f"P {PRIME:x} {PRIME_BYTES}\n".encode())
buf += q.delay_us(50000)
buf += q.mixed_xfer(b"", q.prbs_xorshift32(PRIME, PRIME_BYTES), 0)
buf += q.delay_us(200000)

# ---- DMA: D 32 KiB quad ----
buf += q.uart_tx(f"D {DMA_S:x} {DMA_BYTES}\n".encode())
buf += q.delay_us(50000)
buf += q.mixed_xfer(b"", q.prbs_xorshift32(DMA_S, DMA_BYTES), 0)
buf += q.delay_us(1000000)

with open("step4_primed.qspi", "wb") as f:
    f.write(buf)
print(f"wrote step4_primed.qspi ({len(buf)} bytes)")
