# SPDX-License-Identifier: GPL-3.0
# make_polled_quad.py --- Polled quad RX at 32 KiB (is this a DMA-only bug?)
# Copyright (c) 2026 Jakob Kastelic

import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__),
                                "../../test_serv/examples"))
import make_qspi as q

SEED  = 0x000DECAF
NB    = 32768

buf = bytearray()
buf += q.header(clk_div=q.CLK_DIV_8, mode=q.MODE_QUAD, flags=0)
buf += q.uart_tx(b"M4\n")
buf += q.delay_us(500000)
buf += q.uart_tx(f"P {SEED:x} {NB}\n".encode())
buf += q.delay_us(50000)
buf += q.mixed_xfer(b"", q.prbs_xorshift32(SEED, NB), 0)
buf += q.delay_us(1000000)

with open("polled_quad.qspi", "wb") as f:
    f.write(buf)
print(f"wrote polled_quad.qspi ({len(buf)} bytes)")
