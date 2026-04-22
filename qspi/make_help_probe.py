# SPDX-License-Identifier: GPL-3.0
# make_help_probe.py --- Verify ? / help / unknown commands print usage
# Copyright (c) 2026 Jakob Kastelic

import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__),
                                "../../test_serv/examples"))
import make_qspi as q

buf = bytearray()
buf += q.header(clk_div=q.CLK_DIV_8, mode=q.MODE_SINGLE, flags=0)
buf += q.uart_tx(b"?\n")
buf += q.delay_us(200000)
buf += q.uart_tx(b"help\n")
buf += q.delay_us(200000)
buf += q.uart_tx(b"ZZZ\n")
buf += q.delay_us(500000)

with open("help_probe.qspi", "wb") as f:
    f.write(buf)
print("wrote help_probe.qspi")
