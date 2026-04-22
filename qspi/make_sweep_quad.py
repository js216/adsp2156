# SPDX-License-Identifier: GPL-3.0
# make_sweep_quad.py --- Burst-size sweep for quad slave DMA bringup
# Copyright (c) 2026 Jakob Kastelic

# Drives five quad-lane PRBS bursts at 16/64/256/1024/4096 bytes and
# captures each PRBSDMA result.  Purpose: pinpoint the burst length
# at which lane D1 (PA0) starts reading stuck-at-1 in sustained quad
# slave RX.  Step 3 (64 B) passes; step 4 (>=32 KiB) fails with
# first_at=0x14, so the threshold lives somewhere between.

import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__),
                                "../../test_serv/examples"))
import make_qspi as q

PRBS_SEED = 0x000DECAF
SIZES = [16, 64, 256, 1024, 4096]

DELAY_CMD_US  = 400000
DELAY_ARM_US  = 50000
DELAY_TAIL_US = 200000

buf = bytearray()
buf += q.header(clk_div=q.CLK_DIV_8, mode=q.MODE_QUAD, flags=0)
buf += q.uart_tx(b"M4\n")
buf += q.delay_us(DELAY_CMD_US)

for n in SIZES:
    buf += q.uart_tx(f"D decaf {n}\n".encode())
    buf += q.delay_us(DELAY_ARM_US)
    buf += q.mixed_xfer(b"", q.prbs_xorshift32(PRBS_SEED, n), 0)
    buf += q.delay_us(DELAY_TAIL_US)

with open("sweep_quad.qspi", "wb") as f:
    f.write(buf)
print(f"wrote sweep_quad.qspi ({len(buf)} bytes), sizes={SIZES}")
