# SPDX-License-Identifier: GPL-3.0
# make_step1.py --- Build step1.qspi for TODO.md step 1 (slave x1 polled)
# Copyright (c) 2026 Jakob Kastelic

import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__),
                                "../../test_serv/examples"))
import make_qspi as q

buf = bytearray()
# v1 header: FT4222 master at DIV_8, single lane, flags=0
# (CPOL idle-low, CPHA clock-trailing = CPHA=1 standard).
buf += q.header(clk_div=q.CLK_DIV_8, mode=q.MODE_SINGLE, flags=0)

# Put DSP slave in single-lane mode (also reapplies SLVSEL/FER/CPHA
# via spi_reconfigure), then arm the polled PRBS receive for
# 64 bytes with seed 0xC0FFEE.
buf += q.uart_tx(b"M1\n")
buf += q.delay_us(500000)
buf += q.uart_tx(b"P c0ffee 64\n")
# Give the DSP a few ms to enter the polled receive loop before
# FT4222 starts clocking; spi_rx_flush + RFE-wait must be live
# when the first SCLK edge arrives.
buf += q.delay_us(50000)

# Host clocks 64 bytes of PRBS(0xC0FFEE) out on MOSI as master.
buf += q.write_prbs(0x00C0FFEE, 64)

# Trailing window so the UART OK line makes it through before
# the job ends.
buf += q.delay_us(500000)

with open("step1.qspi", "wb") as f:
    f.write(buf)
print(f"wrote step1.qspi ({len(buf)} bytes)")
