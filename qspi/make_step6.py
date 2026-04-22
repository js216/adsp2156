# SPDX-License-Identifier: GPL-3.0
# make_step6.py --- Step 6 probe: DSP master PRBS transmit
# Copyright (c) 2026 Jakob Kastelic

import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__),
                                "../../test_serv/examples"))
import make_qspi as q

buf = bytearray()
buf += q.header(clk_div=q.CLK_DIV_8, mode=q.MODE_SINGLE, flags=0)

# M1, Rm 9, then P c0ffee 64 (DSP master, PRBS TX).  Generous
# inter-command delays to keep the per-byte UART path reliable.
buf += q.uart_tx(b"M1\n")
buf += q.delay_us(500000)
buf += q.uart_tx(b"Rm 9\n")
buf += q.delay_us(500000)
buf += q.uart_tx(b"P c0ffee 64\n")
buf += q.delay_us(500000)
buf += q.uart_tx(b"Rs\n")
buf += q.delay_us(200000)

with open("step6.qspi", "wb") as f:
    f.write(buf)
print(f"wrote step6.qspi ({len(buf)} bytes)")
