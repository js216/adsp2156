# SPDX-License-Identifier: GPL-3.0
# make_quad_64split.py --- 63 KiB + 1 KiB three ways
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

random.seed(123)
a = bytes(random.getrandbits(8) for _ in range(63 * 1024))
b = bytes(random.getrandbits(8) for _ in range(1  * 1024))

print(f"a 63 KiB : 0x{running_xor(a):08x}")
print(f"b  1 KiB : 0x{running_xor(b):08x}")
print(f"a+b      : 0x{running_xor(a+b):08x}")

buf = bytearray()
buf += q.header(clk_div=q.CLK_DIV_8, mode=q.MODE_QUAD, flags=0)
buf += q.uart_tx(b"m4\r");  buf += q.delay_us(300000)

# ---- Experiment 1: a, i, b, i ----
buf += q.uart_tx(b"--- EXP 1 ---\r");  buf += q.delay_us(100000)
buf += q.uart_tx(b"i\r");   buf += q.delay_us(200000)
buf += q.mixed_xfer(b"", a, 0)
buf += q.delay_us(300000)
buf += q.uart_tx(b"i\r");   buf += q.delay_us(200000)  # sees a
buf += q.mixed_xfer(b"", b, 0)
buf += q.delay_us(300000)
buf += q.uart_tx(b"i\r");   buf += q.delay_us(200000)  # sees b

# ---- Experiment 2: a, b, i ----
buf += q.uart_tx(b"--- EXP 2 ---\r");  buf += q.delay_us(100000)
buf += q.uart_tx(b"i\r");   buf += q.delay_us(200000)  # reset
buf += q.mixed_xfer(b"", a, 0)
buf += q.mixed_xfer(b"", b, 0)
buf += q.delay_us(300000)
buf += q.uart_tx(b"i\r");   buf += q.delay_us(200000)  # sees a+b

# ---- Experiment 3: chunking loop ----
buf += q.uart_tx(b"--- EXP 3 ---\r");  buf += q.delay_us(100000)
buf += q.uart_tx(b"i\r");   buf += q.delay_us(200000)  # reset
payload = a + b
CHUNK = 63 * 1024   # arbitrary split: first chunk 63 KiB, remainder 1 KiB
for off in range(0, len(payload), CHUNK):
    buf += q.mixed_xfer(b"", payload[off:off+CHUNK], 0)
buf += q.delay_us(300000)
buf += q.uart_tx(b"i\r")

with open("quad_64split.qspi", "wb") as f:
    f.write(buf)
print("wrote quad_64split.qspi")
