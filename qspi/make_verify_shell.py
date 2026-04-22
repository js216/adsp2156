# SPDX-License-Identifier: GPL-3.0
# make_verify_shell.py --- Smoke test the new shell: help, B, T, E, H
# Copyright (c) 2026 Jakob Kastelic

import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__),
                                "../../test_serv/examples"))
import make_qspi as q

def cmd(s):
    return q.uart_tx((s + "\r").encode()) + q.delay_us(200000)

buf = bytearray()
buf += q.header(clk_div=q.CLK_DIV_8, mode=q.MODE_SINGLE, flags=0)

# Let the shell print its "ready. type help" and first "> ".
buf += q.delay_us(400000)

buf += cmd("help")
buf += cmd("M1")          # should confirm default
buf += cmd("B 64")        # 64 B buffer -> 16 words per summary
buf += cmd("?")           # state check after B
buf += cmd("E 0")         # echo off -- next line should not echo
buf += cmd("E 1")         # echo back on
buf += cmd("H 64")        # 64 B dump with host PRBS
buf += q.delay_us(50000)
buf += q.write_prbs(0xDECAF, 64)
buf += q.delay_us(500000)

# Try unknown cmd
buf += cmd("ZZZ")

with open("verify_shell.qspi", "wb") as f:
    f.write(buf)
print("wrote verify_shell.qspi")
