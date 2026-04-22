# SPDX-License-Identifier: GPL-3.0
# make_verify_v2.py --- Check I command + 512 KiB max buffer
# Copyright (c) 2026 Jakob Kastelic

import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__),
                                "../../test_serv/examples"))
import make_qspi as q

def cmd(s):
    return q.uart_tx((s + "\r").encode()) + q.delay_us(200000)

buf = bytearray()
buf += q.header(clk_div=q.CLK_DIV_8, mode=q.MODE_SINGLE, flags=0)
buf += q.delay_us(400000)

buf += cmd("help")                  # confirm new 512K line + I doc
buf += cmd("I")                     # level=0 sum=0 when idle
buf += cmd("B 524288")              # max
buf += cmd("I")                     # level=0/524288 sum=0
buf += cmd("B 524292")              # over-max -> ERR
buf += cmd("B 64")                  # back to small
buf += cmd("H 64")                  # trigger fill -> sum=0x072f19e7
buf += q.delay_us(50000)
buf += q.write_prbs(0x000DECAF, 64)
buf += q.delay_us(400000)
buf += cmd("I")                     # level=0, sum reset to 0

with open("verify_v2.qspi", "wb") as f:
    f.write(buf)
print("wrote verify_v2.qspi")
