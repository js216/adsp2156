# SPDX-License-Identifier: GPL-3.0
# make_step7.py --- Step 7 probe: DSP master V (RX+verify PRBS)
# Copyright (c) 2026 Jakob Kastelic

import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__),
                                "../../test_serv/examples"))
import make_qspi as q

buf = bytearray()
buf += q.header(clk_div=q.CLK_DIV_8, mode=q.MODE_SINGLE, flags=0)
buf += q.uart_tx(b"M1\n")
buf += q.delay_us(500000)
buf += q.uart_tx(b"Rm 9\n")
buf += q.delay_us(500000)
buf += q.uart_tx(b"V decaf 64\n")
buf += q.delay_us(500000)
buf += q.uart_tx(b"Rs\n")
buf += q.delay_us(200000)

with open("step7.qspi", "wb") as f:
    f.write(buf)
print(f"wrote step7.qspi ({len(buf)} bytes)")
