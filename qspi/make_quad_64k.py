# SPDX-License-Identifier: GPL-3.0
# make_quad_64k.py --- Reproduce the 64 KiB quad cksum=0 issue
# Copyright (c) 2026 Jakob Kastelic

# User report: 63 KiB works, 64 KiB checksum is zero on firmware.
# TLV nw is u16 (max 65535) so exactly 65536 B cannot fit in one TLV.
# Test three variants:
#   A) 65535 B in one TLV       -- max single-call
#   B) 32768 + 32768 = 64 KiB   -- two TLVs, two CS cycles
#   C) 65535 + 1 = 64 KiB + 0   -- pathological but valid

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

def build(name, chunks):
    data = b"".join(chunks)
    print(f"{name}: total={len(data)} chunks={[len(c) for c in chunks]} "
          f"expected=0x{running_xor(data):08x}")
    buf = bytearray()
    buf += q.header(clk_div=q.CLK_DIV_8, mode=q.MODE_QUAD, flags=0)
    buf += q.uart_tx(b"m4\r");  buf += q.delay_us(300000)
    buf += q.uart_tx(b"i\r");   buf += q.delay_us(200000)
    for c in chunks:
        buf += q.mixed_xfer(b"", c, 0)
    buf += q.delay_us(300000)
    buf += q.uart_tx(b"i\r")
    return buf

# Variant A: single max-TLV (65535 B), multiple of 4? 65535 is not.
# Round down to 65532 = 0xFFFC.
A = [bytes(random.getrandbits(8) for _ in range(65532))]
with open("quad_A_65532.qspi", "wb") as f: f.write(build("A_65532", A))

# Variant B: 2 TLVs * 32 KiB = 64 KiB, two CS cycles.
B = [bytes(random.getrandbits(8) for _ in range(32768)),
     bytes(random.getrandbits(8) for _ in range(32768))]
with open("quad_B_64k_split.qspi", "wb") as f: f.write(build("B_64k_split", B))
