# SPDX-License-Identifier: GPL-3.0
# make_uart_probe.py --- long-delay qspi job to verify UART capture
# Copyright (c) 2026 Jakob Kastelic

import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__),
                                "../../test_serv/examples"))
import make_qspi as q

buf = bytearray()
buf += q.header(clk_div=q.CLK_DIV_8, mode=q.MODE_SINGLE, flags=0)
buf += q.delay_us(2_500_000)

with open("uart_probe.qspi", "wb") as f:
    f.write(buf)
print(f"wrote uart_probe.qspi ({len(buf)} bytes)")
