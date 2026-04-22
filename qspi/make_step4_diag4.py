# SPDX-License-Identifier: GPL-3.0
# make_step4_diag4.py --- Zero in: which bits of word 0 trigger the stuck-high?
# Copyright (c) 2026 Jakob Kastelic

import os, sys
sys.path.insert(0, os.path.join(os.path.dirname(__file__),
                                "../../test_serv/examples"))
import make_qspi as q

DELAY_CMD_US=500000; DELAY_ARM_US=50000; DELAY_BTW_US=200000; DELAY_TAIL_US=1000000
N=64
c = q.prbs_xorshift32(0x00C0FFEE, N)
d = q.prbs_xorshift32(0x000DECAF, N)

buf = bytearray()
buf += q.header(clk_div=q.CLK_DIV_8, mode=q.MODE_QUAD, flags=0)
buf += q.uart_tx(b"M4\n")
buf += q.delay_us(DELAY_CMD_US)

def frame(cmd, payload):
    out = bytearray()
    out += q.uart_tx(cmd)
    out += q.delay_us(DELAY_ARM_US)
    out += q.mixed_xfer(b"", bytes(payload), 0)
    out += q.delay_us(DELAY_BTW_US)
    return out

# Try k=3: replace only bytes 0..2 with decaf. That's 3 mismatches max no-stuck,
# 36 if stuck. Was not in the first diag sweep because we jumped 1,2,4,8..
buf += frame(b"D c0ffee 64\n", d[:3]+c[3:])  # k=3

# Now try: replace ONLY byte 3 (last byte of word 0) with decaf
payload = bytearray(c)
payload[3] = d[3]
buf += frame(b"D c0ffee 64\n", bytes(payload))

# Only byte 2
payload = bytearray(c)
payload[2] = d[2]
buf += frame(b"D c0ffee 64\n", bytes(payload))

# c0ffee first 4 bytes but with any single bit flipped in word 0
# Flip MSB of byte 0 (ie. change c[0]=0x70 to 0xF0)
payload = bytearray(c)
payload[0] ^= 0x80
buf += frame(b"D c0ffee 64\n", bytes(payload))

# Flip LSB of byte 3
payload = bytearray(c)
payload[3] ^= 0x01
buf += frame(b"D c0ffee 64\n", bytes(payload))

# Try: keep first 4 bytes exactly as c0ffee, but replace byte 4 with decaf[4]
# That is, word 0 unchanged, word 1 byte 0 changed.
payload = bytearray(c)
payload[4] = d[4]
buf += frame(b"D c0ffee 64\n", bytes(payload))

# Replace all of word 1 (bytes 4..7) with decaf
payload = bytearray(c); payload[4:8] = d[4:8]
buf += frame(b"D c0ffee 64\n", bytes(payload))

# Replace all of word 4 (bytes 16..19) with decaf (last word before stuck kicks in)
payload = bytearray(c); payload[16:20] = d[16:20]
buf += frame(b"D c0ffee 64\n", bytes(payload))

# Control
buf += frame(b"D c0ffee 64\n", c)

buf += q.delay_us(DELAY_TAIL_US)
with open("step4_diag4.qspi","wb") as f: f.write(buf)
print(f"wrote step4_diag4.qspi ({len(buf)} bytes)")
