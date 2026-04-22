# SPDX-License-Identifier: GPL-3.0
# make_step9.py --- Step 9 probe: master rate sweep
# Copyright (c) 2026 Jakob Kastelic

import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__),
                                "../../test_serv/examples"))
import make_qspi as q

buf = bytearray()
buf += q.header(clk_div=q.CLK_DIV_8, mode=q.MODE_SINGLE, flags=0)
buf += q.uart_tx(b"M1\n")
buf += q.delay_us(500000)
for div in (9, 6, 4):
    buf += q.uart_tx(f"Rm {div}\n".encode())
    buf += q.delay_us(500000)
    buf += q.uart_tx(b"X c0ffee 64\n")
    buf += q.delay_us(500000)
buf += q.uart_tx(b"Rs\n")
buf += q.delay_us(200000)

with open("step9.qspi", "wb") as f:
    f.write(buf)
print(f"wrote step9.qspi ({len(buf)} bytes)")
