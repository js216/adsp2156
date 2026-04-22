# SPDX-License-Identifier: GPL-3.0
# make_stream_512k.py --- Send 512 KiB single-lane, compare XOR checksum
# Copyright (c) 2026 Jakob Kastelic

import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__),
                                "../../test_serv/examples"))
import make_qspi as q

SEED = 0x000DECAF
N    = 512 * 1024

def cmd(s):
    return q.uart_tx((s + "\r").encode()) + q.delay_us(200000)

def running_xor(data: bytes) -> int:
    acc = 0
    for i in range(0, len(data), 4):
        w = (data[i]<<24) | (data[i+1]<<16) | (data[i+2]<<8) | data[i+3]
        acc ^= w
    return acc & 0xFFFFFFFF

data = q.prbs_xorshift32(SEED, N)
print(f"expected sum = 0x{running_xor(data):08x}")

buf = bytearray()
buf += q.header(clk_div=q.CLK_DIV_8, mode=q.MODE_SINGLE, flags=0)
buf += q.delay_us(400000)
buf += cmd("B 524288")
buf += cmd(f"H {N}")
buf += q.delay_us(50000)
buf += q.write_prbs(SEED, N)
buf += q.delay_us(3000000)

with open("stream_512k.qspi", "wb") as f:
    f.write(buf)
print(f"wrote stream_512k.qspi ({len(buf)} bytes)")
