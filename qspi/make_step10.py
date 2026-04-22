# SPDX-License-Identifier: GPL-3.0
# make_step10.py --- Step 10 probe: master PRBS burst-size sweep
# Copyright (c) 2026 Jakob Kastelic

import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__),
                                "../../test_serv/examples"))
import make_qspi as q

buf = bytearray()
buf += q.header(clk_div=q.CLK_DIV_8, mode=q.MODE_SINGLE, flags=0)
buf += q.uart_tx(b"M1\n")
buf += q.delay_us(500000)
# Stay at Rm 6 since Rm 4 hangs on this part.
buf += q.uart_tx(b"Rm 6\n")
buf += q.delay_us(500000)
for n in (64, 4096, 16384, 65536):
    buf += q.uart_tx(f"P c0ffee {n}\n".encode())
    # Allow enough time for the largest burst at 13 MHz:
    # 65536 bytes / (13e6 / 8) = ~40 ms plus overhead.
    buf += q.delay_us(500000)
buf += q.uart_tx(b"Rs\n")
buf += q.delay_us(200000)

with open("step10.qspi", "wb") as f:
    f.write(buf)
print(f"wrote step10.qspi ({len(buf)} bytes)")
