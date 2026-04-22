# SPDX-License-Identifier: GPL-3.0
# make_step4_diag3.py --- Narrow down which prefix byte triggers stuck-high
# Copyright (c) 2026 Jakob Kastelic
# Hybrid payloads: replace first k bytes of c0ffee with first k bytes of
# decaf, keep the tail as c0ffee. If after replacing k bytes the mismatch
# count jumps to "stuck" prediction, k is sufficient prefix to trigger.

import os, sys
sys.path.insert(0, os.path.join(os.path.dirname(__file__),
                                "../../test_serv/examples"))
import make_qspi as q

DELAY_CMD_US = 500000
DELAY_ARM_US = 50000
DELAY_BTW_US = 200000
DELAY_TAIL_US = 1000000

N = 64
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

# replace first k bytes of c0ffee with decaf prefix
for k in (1, 2, 4, 8, 12, 16):
    payload = d[:k] + c[k:]
    buf += frame(b"D c0ffee 64\n", payload)

buf += q.delay_us(DELAY_TAIL_US)

with open("step4_diag3.qspi","wb") as f:
    f.write(buf)
print(f"wrote step4_diag3.qspi ({len(buf)} bytes)")
