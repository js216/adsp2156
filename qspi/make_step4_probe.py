# SPDX-License-Identifier: GPL-3.0
# make_step4_probe.py --- Single-TLV quad burst to isolate CS-gap vs rate bugs
# Copyright (c) 2026 Jakob Kastelic

# Step 4 (1 MiB, 32 TLVs) fails with ~14.5% byte mismatches starting
# at byte 20. This probe drives one TLV of 32 KiB (one CS-framed burst
# from FT4222, no intra-stream CS gap) so we can tell whether the bug
# lives in TLV boundaries or in sustained quad-lane slave RX itself.

import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__),
                                "../../test_serv/examples"))
import make_qspi as q

PRBS_SEED  = 0x000DECAF
PRBS_BYTES = 32768

DELAY_CMD_US  = 500000
DELAY_ARM_US  = 50000
DELAY_TAIL_US = 1000000

buf = bytearray()
buf += q.header(clk_div=q.CLK_DIV_8, mode=q.MODE_QUAD, flags=0)

buf += q.uart_tx(b"M4\n")
buf += q.delay_us(DELAY_CMD_US)
buf += q.uart_tx(f"D decaf {PRBS_BYTES}\n".encode())
buf += q.delay_us(DELAY_ARM_US)

buf += q.mixed_xfer(b"", q.prbs_xorshift32(PRBS_SEED, PRBS_BYTES), 0)

buf += q.delay_us(DELAY_TAIL_US)

with open("step4_probe.qspi", "wb") as f:
    f.write(buf)
print(f"wrote step4_probe.qspi ({len(buf)} bytes), 1 TLV {PRBS_BYTES} B")
