# SPDX-License-Identifier: GPL-3.0
# make_quad_tiny_polled.py --- Fine quad sweep, polled path
# Copyright (c) 2026 Jakob Kastelic

import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__),
                                "../../test_serv/examples"))
import make_qspi as q

SEED  = 0x000DECAF
SIZES = [4, 8, 12, 16, 20, 24, 28, 32, 40, 48, 56, 64]

buf = bytearray()
buf += q.header(clk_div=q.CLK_DIV_8, mode=q.MODE_QUAD, flags=0)
buf += q.uart_tx(b"M4\n")
buf += q.delay_us(300000)

for n in SIZES:
    buf += q.uart_tx(f"P {SEED:x} {n}\n".encode())
    buf += q.delay_us(50000)
    buf += q.mixed_xfer(b"", q.prbs_xorshift32(SEED, n), 0)
    buf += q.delay_us(100000)

with open("quad_tiny_polled.qspi", "wb") as f:
    f.write(buf)
print(f"wrote quad_tiny_polled.qspi ({len(buf)} bytes)")
