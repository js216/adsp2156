# SPDX-License-Identifier: GPL-3.0
# make_dual_32k.py --- Dual-lane slave DMA at 32 KiB and 1 MiB
# Copyright (c) 2026 Jakob Kastelic

import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__),
                                "../../test_serv/examples"))
import make_qspi as q

SEED = 0x000DECAF

buf = bytearray()
buf += q.header(clk_div=q.CLK_DIV_8, mode=q.MODE_DUAL, flags=0)
buf += q.uart_tx(b"M2\n")
buf += q.delay_us(500000)

# 32 KiB dual
N = 32768
buf += q.uart_tx(f"D {SEED:x} {N}\n".encode())
buf += q.delay_us(50000)
buf += q.mixed_xfer(b"", q.prbs_xorshift32(SEED, N), 0)
buf += q.delay_us(400000)

# 1 MiB dual (chunked into 32 KiB TLVs due to u16 size limit)
N = 1024 * 1024
buf += q.uart_tx(f"D {SEED:x} {N}\n".encode())
buf += q.delay_us(50000)
full = q.prbs_xorshift32(SEED, N)
CHUNK = 32768
for off in range(0, N, CHUNK):
    buf += q.mixed_xfer(b"", full[off:off + CHUNK], 0)
buf += q.delay_us(2000000)

with open("dual_32k.qspi", "wb") as f:
    f.write(buf)
print(f"wrote dual_32k.qspi ({len(buf)} bytes)")
