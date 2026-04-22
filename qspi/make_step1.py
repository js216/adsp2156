# SPDX-License-Identifier: GPL-3.0
# make_step1.py --- Build step1.qspi for TODO.md step 1 (slave x1 polled)
# Copyright (c) 2026 Jakob Kastelic

import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__),
                                "../../test_serv/examples"))
import make_qspi as q

buf = bytearray()
buf += q.header(clk_div=q.CLK_DIV_8, mode=q.MODE_SINGLE, flags=0)
# Skip all UART commands -- DSP boots into a spy loop that
# auto-prints any SPI RX bytes over UART.  Just clock bytes
# out and see what the slave captured.
buf += q.delay_us(100000)
buf += q.write_prbs(0x00C0FFEE, 64)
buf += q.delay_us(500000)

with open("step1.qspi", "wb") as f:
    f.write(buf)
print(f"wrote step1.qspi ({len(buf)} bytes)")
