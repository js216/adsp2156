# SPDX-License-Identifier: GPL-3.0
# make_quad_chunks.py --- Chunk-count sweep: reset, chunk*N, verify
# Copyright (c) 2026 Jakob Kastelic

import os, random, sys
sys.path.insert(0, os.path.join(os.path.dirname(__file__),
                                "../../test_serv/examples"))
import make_qspi as q

def running_xor(data):
    acc = 0
    for i in range(0, len(data), 4):
        w = (data[i]<<24)|(data[i+1]<<16)|(data[i+2]<<8)|data[i+3]
        acc ^= w
    return acc & 0xFFFFFFFF

CHUNK = 32768
Ns    = [1, 2, 4, 8, 16, 32]

buf = bytearray()
buf += q.header(clk_div=q.CLK_DIV_8, mode=q.MODE_QUAD, flags=0)
buf += q.uart_tx(b"m4\r");  buf += q.delay_us(300000)

random.seed(42)

for N in Ns:
    data = bytes(random.getrandbits(8) for _ in range(N * CHUNK))
    print(f"N={N:2d} total={len(data):>7d} expected=0x{running_xor(data):08x}")
    buf += q.uart_tx(b"i\r");   buf += q.delay_us(100000)  # reset
    for off in range(0, len(data), CHUNK):
        buf += q.mixed_xfer(b"", data[off:off+CHUNK], 0)
    buf += q.delay_us(400000)
    buf += q.uart_tx(b"i\r")
    buf += q.delay_us(200000)

with open("quad_chunks.qspi", "wb") as f:
    f.write(buf)
print(f"wrote quad_chunks.qspi ({len(buf)} bytes)")
