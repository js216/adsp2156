# SPDX-License-Identifier: GPL-3.0
# make_spy.py --- Step 0.5 probe: write 64 PRBS bytes, let DSP SPY
# Copyright (c) 2026 Jakob Kastelic

import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__),
                                "../../test_serv/examples"))
import make_qspi as q

buf = bytearray()
buf += q.header(clk_div=q.CLK_DIV_8, mode=q.MODE_SINGLE, flags=0)
buf += q.delay_us(100000)
buf += q.write_prbs(0x00C0FFEE, 64)
buf += q.delay_us(500000)

with open("spy.qspi", "wb") as f:
    f.write(buf)
print(f"wrote spy.qspi ({len(buf)} bytes)")
