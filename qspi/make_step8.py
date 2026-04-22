# SPDX-License-Identifier: GPL-3.0
# make_step8.py --- Step 8 probe: DSP master X (full-duplex PRBS)
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
buf += q.uart_tx(b"X c0ffee 64\n")
buf += q.delay_us(500000)
buf += q.uart_tx(b"Rs\n")
buf += q.delay_us(200000)

with open("step8.qspi", "wb") as f:
    f.write(buf)
print(f"wrote step8.qspi ({len(buf)} bytes)")
