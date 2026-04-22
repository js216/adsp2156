# SPDX-License-Identifier: GPL-3.0
# make_quad_tiny.py --- Fine-grained quad DMA size sweep
# Copyright (c) 2026 Jakob Kastelic

# Previous coarse sweep: 16 B quad OK, 64 B quad FAIL at byte 20.
# Narrow the onset.  DSP SIZE=32 means count must be a multiple of 4.

import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__),
                                "../../test_serv/examples"))
import make_qspi as q

SEED  = 0x000DECAF
SIZES = [4, 8, 12, 16, 20, 24, 28, 32, 40, 48, 56, 64]

DELAY_CMD_US  = 300000
DELAY_ARM_US  = 50000
DELAY_TAIL_US = 100000

buf = bytearray()
buf += q.header(clk_div=q.CLK_DIV_8, mode=q.MODE_QUAD, flags=0)
buf += q.uart_tx(b"M4\n")
buf += q.delay_us(DELAY_CMD_US)

for n in SIZES:
    buf += q.uart_tx(f"D {SEED:x} {n}\n".encode())
    buf += q.delay_us(DELAY_ARM_US)
    buf += q.mixed_xfer(b"", q.prbs_xorshift32(SEED, n), 0)
    buf += q.delay_us(DELAY_TAIL_US)

with open("quad_tiny.qspi", "wb") as f:
    f.write(buf)
print(f"wrote quad_tiny.qspi ({len(buf)} bytes), sizes={SIZES}")
