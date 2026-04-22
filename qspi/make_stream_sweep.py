# SPDX-License-Identifier: GPL-3.0
# make_stream_sweep.py --- 512 KiB single-lane at DIV_8, DIV_4, DIV_2
# Copyright (c) 2026 Jakob Kastelic

import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__),
                                "../../test_serv/examples"))
import make_qspi as q

SEED = 0x000DECAF
N    = 512 * 1024

RATES = [
    (q.CLK_DIV_8, "DIV_8 (10 MHz)"),
    (q.CLK_DIV_4, "DIV_4 (20 MHz)"),
    (q.CLK_DIV_2, "DIV_2 (40 MHz)"),
]

def cmd(s):
    return q.uart_tx((s + "\r").encode()) + q.delay_us(200000)

def running_xor(data):
    acc = 0
    for i in range(0, len(data), 4):
        w = (data[i]<<24) | (data[i+1]<<16) | (data[i+2]<<8) | data[i+3]
        acc ^= w
    return acc & 0xFFFFFFFF

data = q.prbs_xorshift32(SEED, N)
print(f"expected sum = 0x{running_xor(data):08x}")

buf = bytearray()
buf += q.header(clk_div=RATES[0][0], mode=q.MODE_SINGLE, flags=0)
buf += q.delay_us(400000)
buf += cmd("V 0")
buf += cmd("B 524288")

for i, (clk, name) in enumerate(RATES):
    if i > 0:
        buf += q.reinit(clk, q.MODE_SINGLE, 0)
    buf += cmd(f"H {N}")
    buf += q.delay_us(50000)
    buf += q.write_prbs(SEED, N)
    buf += q.delay_us(800000)

with open("stream_sweep.qspi", "wb") as f:
    f.write(buf)
print(f"wrote stream_sweep.qspi ({len(buf)} bytes)")
