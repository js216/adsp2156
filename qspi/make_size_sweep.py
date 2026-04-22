# SPDX-License-Identifier: GPL-3.0
# make_size_sweep.py --- Quad-slave DMA RX size sweep diagnostic
# Copyright (c) 2026 Jakob Kastelic

# Sweeps the DMA RX size in M4 (quad-slave) to localise the exact
# threshold at which the sustained-frame receive starts failing.
# decaf seed throughout; content is a known red herring (prior
# step-3 M4 PASS with c0ffee + step-4 M4 FAIL with decaf on the
# same 1 MiB code path would not be possible if the bug were
# seed-dependent at the physical layer).

import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__),
                                "../../test_serv/examples"))
import make_qspi as q

DELAY_CMD_US = 500000
DELAY_ARM_US = 50000

SIZES = [16, 20, 24, 32, 48, 64, 128, 256, 512, 1024, 4096, 16384]

buf = bytearray()
# Start directly in QUAD so FT4222 master lanes are wired for x4.
buf += q.header(clk_div=q.CLK_DIV_8, mode=q.MODE_QUAD, flags=0)

buf += q.uart_tx(b"M4\n")
buf += q.delay_us(DELAY_CMD_US)

for n in SIZES:
    buf += q.uart_tx(f"D decaf {n}\n".encode())
    buf += q.delay_us(DELAY_ARM_US)
    buf += q.write_prbs_multi(0x000DECAF, n)
    buf += q.delay_us(DELAY_CMD_US)

with open("size_sweep.qspi", "wb") as f:
    f.write(buf)
print(f"wrote size_sweep.qspi ({len(buf)} bytes, {len(SIZES)} sizes)")
