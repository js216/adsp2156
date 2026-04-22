# SPDX-License-Identifier: GPL-3.0
# make_rate_x1.py --- Single-lane slave RX throughput vs SCK rate
# Copyright (c) 2026 Jakob Kastelic

# One big single-lane PRBS burst per SCK-divisor setting.  The DSP
# reports ticks; divide by SCLK_TICKS_PER_MSEC=93750 for ms.  The
# FT4222 default OpClk is 60 MHz, so DIV_N gives SCK = 60 MHz / N.
# Poller already chunks the write in 16 KiB slices keeping CS low
# (spiMaster_SingleWrite with isEndTransaction=False), so the DSP
# sees one continuous CS-framed burst per D command.

import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__),
                                "../../test_serv/examples"))
import make_qspi as q

SEED  = 0x000DECAF
NBYTE = 1024 * 1024

# DIV_8, DIV_4, DIV_2.  q.CLK_DIV_* constants pick these.  DIV_2 at
# default 60 MHz OpClk = 30 MHz SCK, which is within FT4222 master
# max (30 MHz) and well under the DSP slave's SPI_CLK/2 ceiling.
RATES = [
    (q.CLK_DIV_8, "DIV_8",  "7.5 MHz"),
    (q.CLK_DIV_4, "DIV_4",  "15 MHz"),
    (q.CLK_DIV_2, "DIV_2",  "30 MHz"),
]

buf = bytearray()
# Start at DIV_8 single; later iterations reinit.
first_clk, _, _ = RATES[0]
buf += q.header(clk_div=first_clk, mode=q.MODE_SINGLE, flags=0)
buf += q.uart_tx(b"M1\n")
buf += q.delay_us(400000)

for i, (clk, name, hz) in enumerate(RATES):
    if i > 0:
        buf += q.reinit(clk, q.MODE_SINGLE, 0)
    buf += q.uart_tx(f"D {SEED:x} {NBYTE}\n".encode())
    buf += q.delay_us(50000)
    buf += q.write_prbs(SEED, NBYTE)
    buf += q.delay_us(400000)

with open("rate_x1.qspi", "wb") as f:
    f.write(buf)
print(f"wrote rate_x1.qspi ({len(buf)} bytes), rates="
      f"{[name for _, name, _ in RATES]}")
