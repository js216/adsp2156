# SPDX-License-Identifier: GPL-3.0
# make_step5.py --- Step 5 probe: DSP as master, clock 64 zero bytes
# Copyright (c) 2026 Jakob Kastelic

import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__),
                                "../../test_serv/examples"))
import make_qspi as q

buf = bytearray()
# v1 header: poller sets FT4222 as master at DIV_8.  That conflicts
# with DSP-as-master on the same pins, but for this step the FT4222
# just needs to be idle so DSP drives the bus unchallenged.  Do not
# issue any SPI opcode; only send UART commands.
buf += q.header(clk_div=q.CLK_DIV_8, mode=q.MODE_SINGLE, flags=0)

# Put DSP in single-lane mode, switch role to master at clkdiv=9
# (SCLK0 / (9+1) = 9.375 MHz), then clock 64 zero bytes out.
buf += q.uart_tx(b"M1\n")
buf += q.delay_us(500000)
buf += q.uart_tx(b"Rm 9\n")
buf += q.delay_us(500000)
buf += q.uart_tx(b"L 64\n")
buf += q.delay_us(500000)

# Switch back to slave so subsequent jobs start from a known state.
buf += q.uart_tx(b"Rs\n")
buf += q.delay_us(200000)

with open("step5.qspi", "wb") as f:
    f.write(buf)
print(f"wrote step5.qspi ({len(buf)} bytes)")
