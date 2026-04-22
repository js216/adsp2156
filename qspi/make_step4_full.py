# SPDX-License-Identifier: GPL-3.0
# make_step4_full.py --- Step-3 preamble + large DMA as workaround probe
# Copyright (c) 2026 Jakob Kastelic

# Step 3's M4 DMA passes at 64 B.  Probe: reproduce the full step-3
# preamble (M2 P D, reinit, M4 P D with small size) then retry DMA
# at 32 KiB to see whether the prior x2+x4 activity primes the
# peripheral enough for sustained quad RX.

import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__),
                                "../../test_serv/examples"))
import make_qspi as q

buf = bytearray()
buf += q.header(clk_div=q.CLK_DIV_8, mode=q.MODE_DUAL, flags=0)

# ---- x2 warm-up ----
buf += q.uart_tx(b"M2\n")
buf += q.delay_us(400000)
buf += q.uart_tx(b"P c0ffee 64\n")
buf += q.delay_us(50000)
buf += q.mixed_xfer(b"", q.prbs_xorshift32(0x00C0FFEE, 64), 0)
buf += q.delay_us(300000)

buf += q.uart_tx(b"D c0ffee 64\n")
buf += q.delay_us(50000)
buf += q.mixed_xfer(b"", q.prbs_xorshift32(0x00C0FFEE, 64), 0)
buf += q.delay_us(300000)

# ---- switch host to quad ----
buf += q.reinit(q.CLK_DIV_8, q.MODE_QUAD, 0)

# ---- x4 warm-up ----
buf += q.uart_tx(b"M4\n")
buf += q.delay_us(400000)
buf += q.uart_tx(b"P c0ffee 64\n")
buf += q.delay_us(50000)
buf += q.mixed_xfer(b"", q.prbs_xorshift32(0x00C0FFEE, 64), 0)
buf += q.delay_us(300000)

buf += q.uart_tx(b"D c0ffee 64\n")
buf += q.delay_us(50000)
buf += q.mixed_xfer(b"", q.prbs_xorshift32(0x00C0FFEE, 64), 0)
buf += q.delay_us(300000)

# ---- the actual 32 KiB stress test ----
buf += q.uart_tx(b"D decaf 32768\n")
buf += q.delay_us(50000)
buf += q.mixed_xfer(b"", q.prbs_xorshift32(0x000DECAF, 32768), 0)
buf += q.delay_us(1000000)

with open("step4_full.qspi", "wb") as f:
    f.write(buf)
print(f"wrote step4_full.qspi ({len(buf)} bytes)")
