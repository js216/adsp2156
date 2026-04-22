# SPDX-License-Identifier: GPL-3.0
# make_step4_diag.py --- Diagnostic decaf vs c0ffee; small vs large chunks
# Copyright (c) 2026 Jakob Kastelic

# Step 4 diagnostic. Isolates three discriminators in one UART capture:
#
#   (1) repeat Step 3's canonical "M4 + D c0ffee 64 + mixed_xfer(b"",
#       prbs_c0ffee_64, 0)" inside step4.qspi's exact framing. This is
#       the control: it must still print OK.
#
#   (2) change the seed to decaf, keep everything else identical. If
#       this prints OK the "seed-specific D1-stuck-high" theory
#       collapses; if it FAILs with mismatches in bytes 20..63 the
#       theory is reproducible.
#
#   (3) run one 4 KiB c0ffee frame. Between 64 B (Step 3 PASS) and 32
#       KiB (Step 4 FAIL) lies some threshold. 4 KiB is well above
#       byte 20 and still comfortably inside every internal FT4222
#       buffer + USB endpoint size, so if the bug were a "frame size
#       >= threshold" property, 4 KiB would already show it.

import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__),
                                "../../test_serv/examples"))
import make_qspi as q

DELAY_CMD_US  = 500000
DELAY_ARM_US  = 50000
DELAY_BTW_US  = 200000
DELAY_TAIL_US = 1000000

SMALL_N = 64
LARGE_N = 4096

buf = bytearray()
buf += q.header(clk_div=q.CLK_DIV_8, mode=q.MODE_QUAD, flags=0)
buf += q.uart_tx(b"M4\n")
buf += q.delay_us(DELAY_CMD_US)

# (1) c0ffee 64
buf += q.uart_tx(b"D c0ffee 64\n")
buf += q.delay_us(DELAY_ARM_US)
buf += q.mixed_xfer(b"", q.prbs_xorshift32(0x00C0FFEE, SMALL_N), 0)
buf += q.delay_us(DELAY_BTW_US)

# (2) decaf 64
buf += q.uart_tx(b"D decaf 64\n")
buf += q.delay_us(DELAY_ARM_US)
buf += q.mixed_xfer(b"", q.prbs_xorshift32(0x000DECAF, SMALL_N), 0)
buf += q.delay_us(DELAY_BTW_US)

# (3) c0ffee 4096
buf += q.uart_tx(f"D c0ffee {LARGE_N}\n".encode())
buf += q.delay_us(DELAY_ARM_US)
buf += q.mixed_xfer(b"", q.prbs_xorshift32(0x00C0FFEE, LARGE_N), 0)
buf += q.delay_us(DELAY_BTW_US)

# (4) decaf 4096
buf += q.uart_tx(f"D decaf {LARGE_N}\n".encode())
buf += q.delay_us(DELAY_ARM_US)
buf += q.mixed_xfer(b"", q.prbs_xorshift32(0x000DECAF, LARGE_N), 0)

buf += q.delay_us(DELAY_TAIL_US)

with open("step4_diag.qspi", "wb") as f:
    f.write(buf)
print(f"wrote step4_diag.qspi ({len(buf)} bytes)")
