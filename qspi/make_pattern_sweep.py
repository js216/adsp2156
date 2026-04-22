# SPDX-License-Identifier: GPL-3.0
# make_pattern_sweep.py --- Content-pattern sweep to isolate D1 failure
# Copyright (c) 2026 Jakob Kastelic

import os, random, sys
sys.path.insert(0, os.path.join(os.path.dirname(__file__),
                                "../../test_serv/examples"))
import make_qspi as q

SIZE = 63 * 1024   # 63 KiB payloads for the long patterns

def running_xor(d):
    a = 0
    for i in range(0, len(d), 4):
        a ^= (d[i]<<24)|(d[i+1]<<16)|(d[i+2]<<8)|d[i+3]
    return a & 0xFFFFFFFF

random.seed(123);  s123 = bytes(random.getrandbits(8) for _ in range(SIZE))
random.seed(999);  s999 = bytes(random.getrandbits(8) for _ in range(SIZE))

patterns = [
    # Test 0: 10 words seed-999 + 10 words seed-123 (80 B total, 1 CS)
    ("0: s999[0:40] + s123[0:40]",    s999[:40] + s123[:40]),
    # Test 1: all zeros (D1=0 everywhere)
    ("1: all 0x00, 63 KiB",           bytes(SIZE)),
    # Test 2: all 0xFF (D1=1 everywhere -- maximum stress for stuck-at-0)
    ("2: all 0xFF, 63 KiB",           bytes([0xFF] * SIZE)),
    # Test 3: alternating 0xAA/0x55 (D1 toggles every bit -- max transition)
    ("3: alt AA/55, 63 KiB",          bytes([0xAA, 0x55] * (SIZE // 2))),
    # Test 4: all 0x22 (only D1 bits set, D0/D2/D3 all 0)
    ("4: all 0x22, 63 KiB",           bytes([0x22] * SIZE)),
    # Test 5: all 0xDD (D1 clear everywhere, D0/D2/D3 set)
    ("5: all 0xDD, 63 KiB",           bytes([0xDD] * SIZE)),
    # Test 6: incrementing byte counter
    ("6: counter 0..255, 63 KiB",     bytes([i & 0xFF for i in range(SIZE)])),
    # Test 7: D1=1 on nibble 0 only (0x02 bytes) -- sparse D1 activity
    ("7: all 0x02, 63 KiB",           bytes([0x02] * SIZE)),
    # Test 8: D1 long runs: 32 bytes of D1=1 then 32 of D1=0
    ("8: D1 run 32/32, 63 KiB",
        (bytes([0x22]*32) + bytes([0x00]*32)) * (SIZE // 64)),
    # Test 9: isolate first failure point -- just w0..w9 of s123
    ("9: s123[0:40], 10 words",       s123[:40]),
]

buf = bytearray()
buf += q.header(clk_div=q.CLK_DIV_8, mode=q.MODE_QUAD, flags=0)
buf += q.uart_tx(b"m4\r");  buf += q.delay_us(300000)

for (name, data) in patterns:
    expected = running_xor(data)
    print(f"{name:40s} len={len(data):6d} expected=0x{expected:08x}")
    buf += q.uart_tx(b"i\r");   buf += q.delay_us(150000)  # reset
    buf += q.mixed_xfer(b"", data, 0)
    buf += q.delay_us(150000)
    buf += q.uart_tx(b"i\r");   buf += q.delay_us(150000)  # report

with open("pattern_sweep.qspi", "wb") as f:
    f.write(buf)
print(f"\nwrote pattern_sweep.qspi")
