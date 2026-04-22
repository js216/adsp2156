# SPDX-License-Identifier: GPL-3.0
# make_rate_x2.py --- Dual-lane slave RX throughput vs SCK rate
# Copyright (c) 2026 Jakob Kastelic

# Dual is forced through mixed_xfer (spiMaster_MultiReadWrite) which
# has a 65535 B per-call cap, so 1 MiB comes across as 32 TLVs of
# 32 KiB each.  Each TLV ends with CS de-assertion -- there is no
# "CS held across MultiReadWrite" idiom in libFT4222.  This matches
# the step-4 pattern.

import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__),
                                "../../test_serv/examples"))
import make_qspi as q

SEED  = 0x000DECAF
NBYTE = 1024 * 1024
CHUNK = 32768

RATES = [
    (q.CLK_DIV_8, "DIV_8"),
    (q.CLK_DIV_4, "DIV_4"),
    (q.CLK_DIV_2, "DIV_2"),
]

full = q.prbs_xorshift32(SEED, NBYTE)

buf = bytearray()
first_clk, _ = RATES[0]
buf += q.header(clk_div=first_clk, mode=q.MODE_DUAL, flags=0)
buf += q.uart_tx(b"M2\n")
buf += q.delay_us(400000)

for i, (clk, name) in enumerate(RATES):
    if i > 0:
        buf += q.reinit(clk, q.MODE_DUAL, 0)
    buf += q.uart_tx(f"D {SEED:x} {NBYTE}\n".encode())
    buf += q.delay_us(50000)
    for off in range(0, NBYTE, CHUNK):
        buf += q.mixed_xfer(b"", full[off:off + CHUNK], 0)
    buf += q.delay_us(500000)

with open("rate_x2.qspi", "wb") as f:
    f.write(buf)
print(f"wrote rate_x2.qspi ({len(buf)} bytes), rates="
      f"{[name for _, name in RATES]}")
