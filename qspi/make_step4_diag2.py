# SPDX-License-Identifier: GPL-3.0
# make_step4_diag2.py --- Seed-content discriminator for Step 4 FAIL
# Copyright (c) 2026 Jakob Kastelic

# Discriminator. Known: decaf 64B fails at byte 20; c0ffee 64B passes.
# Test whether the failure is caused by the first 20 bytes (prefix
# theory) or by bytes 20+ (suffix theory) by splicing the two
# streams:
#
#   T1: send decaf-bytes[0..20) + c0ffee-bytes[20..64). DSP is told
#       "verify against c0ffee". If first_at=0 and not 20, the
#       prefix made no difference to byte 20 behaviour. If bytes 0
#       through 19 mismatch (20 of them) but 20..63 pass, the
#       FAILURE window is bytes 0..19 (prefix itself didn't "put
#       the chip in stuck mode"). If we see stuck-high starting at
#       byte 20 again, the prefix primed the chip.
#
#   T2: send c0ffee[0..20) + decaf[20..64). DSP verifies against
#       c0ffee. If bytes 0..19 pass and 20..63 all mismatch, that
#       just reflects the expected seed mismatch on 20..63.  What
#       tells us something is whether *stuck-high* shows: compute
#       expected result vs 'got=c0ffee|0x22' and see how many of the
#       reported mismatches are the OR-0x22 pattern.
#
#   T3: send all-zeros 64 B (MOSI driving 0). DSP verifies against
#       c0ffee (knowing it will mismatch). If the returned bytes are
#       'expected|0x22' shape, stuck-high triggered with zero
#       payload too. If they're 'got=0', stuck-high didn't trigger.
#
#   T4: send all-0xFF 64 B. Same logic for "stuck-low" mirror.

import os
import sys

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

def frame(label, cmd, payload):
    out = bytearray()
    out += q.uart_tx(cmd)
    out += q.delay_us(DELAY_ARM_US)
    out += q.mixed_xfer(b"", bytes(payload), 0)
    out += q.delay_us(DELAY_BTW_US)
    return out

# T1: decaf[0:20] + c0ffee[20:64], verify vs c0ffee
buf += frame("T1", b"D c0ffee 64\n", d[:20] + c[20:])

# T2: c0ffee[0:20] + decaf[20:64], verify vs c0ffee
buf += frame("T2", b"D c0ffee 64\n", c[:20] + d[20:])

# T3: all zeros, verify vs c0ffee (so everything mismatches; look at
# first_at and whether reported bytes are 0x00 or 0x22)
buf += frame("T3", b"D c0ffee 64\n", bytes(64))

# T4: all 0xFF, verify vs c0ffee
buf += frame("T4", b"D c0ffee 64\n", bytes([0xFF]*64))

# T5: decaf[0:24] + c0ffee[24:64], verify vs c0ffee -- probes whether
# even one extra word of decaf prefix keeps the stuck-high alive
buf += frame("T5", b"D c0ffee 64\n", d[:24] + c[24:])

# Control (must PASS)
buf += frame("CTL", b"D c0ffee 64\n", c)

buf += q.delay_us(DELAY_TAIL_US)

with open("step4_diag2.qspi", "wb") as f:
    f.write(buf)
print(f"wrote step4_diag2.qspi ({len(buf)} bytes)")
