# SPDX-License-Identifier: GPL-3.0
# make_quad_diag.py --- Dump dma_rx_buf for both seed-123 (fail) and
# seed-999 (pass) payloads, so the host-side diff shows which bytes
# actually diverge on the failing content.
# Copyright (c) 2026 Jakob Kastelic

import os, random, sys
sys.path.insert(0, os.path.join(os.path.dirname(__file__),
                                "../../test_serv/examples"))
import make_qspi as q

SIZE = 63 * 1024

for seed in (123, 999):
    random.seed(seed)
    data = bytes(random.getrandbits(8) for _ in range(SIZE))

    with open(f"quad_diag_exp_s{seed}.bin", "wb") as f:
        f.write(data)

    buf = bytearray()
    buf += q.header(clk_div=q.CLK_DIV_8, mode=q.MODE_QUAD, flags=0)
    buf += q.uart_tx(b"m4\r");  buf += q.delay_us(300000)
    buf += q.uart_tx(b"i\r");   buf += q.delay_us(200000)
    buf += q.mixed_xfer(b"", data, 0)
    buf += q.delay_us(200000)
    # h prints every word as "w<idx>=0x<hex>\r\n" -- ~22 B/line,
    # 16128 lines at 115200 baud = ~32 s.
    buf += q.uart_tx(f"h {SIZE}\r".encode())
    buf += q.delay_us(35000000)

    fn = f"quad_diag_s{seed}.qspi"
    with open(fn, "wb") as f: f.write(buf)
    print(f"wrote {fn}  (seed={seed}, size={SIZE})")
