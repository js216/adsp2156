# SPDX-License-Identifier: GPL-3.0
# make_single_sweep.py --- Single-lane DMA sweep: 64K, 256K, 512K, 1M
# Copyright (c) 2026 Jakob Kastelic

import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__),
                                "../../test_serv/examples"))
import make_qspi as q

SEED = 0x000DECAF

buf = bytearray()
buf += q.header(clk_div=q.CLK_DIV_8, mode=q.MODE_SINGLE, flags=0)
buf += q.uart_tx(b"M1\n")
buf += q.delay_us(500000)

for N in (65536, 262144, 524288):
    buf += q.uart_tx(f"D {SEED:x} {N}\n".encode())
    buf += q.delay_us(50000)
    buf += q.write_prbs(SEED, N)
    buf += q.delay_us(500000)

with open("single_sweep.qspi", "wb") as f:
    f.write(buf)
print(f"wrote single_sweep.qspi ({len(buf)} bytes)")
