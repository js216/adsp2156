# SPDX-License-Identifier: GPL-3.0
# make_step4_x1.py --- 32 KiB single-lane DMA probe (isolate rate vs lane)
# Copyright (c) 2026 Jakob Kastelic

import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__),
                                "../../test_serv/examples"))
import make_qspi as q

PRBS_SEED  = 0x000DECAF
PRBS_BYTES = 32768

buf = bytearray()
buf += q.header(clk_div=q.CLK_DIV_8, mode=q.MODE_SINGLE, flags=0)
buf += q.uart_tx(b"M1\n")
buf += q.delay_us(500000)
buf += q.uart_tx(f"D decaf {PRBS_BYTES}\n".encode())
buf += q.delay_us(50000)
buf += q.write_prbs(PRBS_SEED, PRBS_BYTES)
buf += q.delay_us(1000000)

with open("step4_x1.qspi", "wb") as f:
    f.write(buf)
print(f"wrote step4_x1.qspi ({len(buf)} bytes)")
