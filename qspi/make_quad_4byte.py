# SPDX-License-Identifier: GPL-3.0
# make_quad_4byte.py --- 4 bytes per quad CS frame, dump DSP RX
# Copyright (c) 2026 Jakob Kastelic

# Each host TLV is exactly one 32-bit word -- shortest unit the DSP
# can latch.  16 TLVs -> 16 CS frames -> 16 words.  DSP H command
# dumps every one, unverified.  Lets us see whether short CS frames
# evade the byte-20 quad-slave corruption.

import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__),
                                "../../test_serv/examples"))
import make_qspi as q

NWORDS = 16
NB     = 4 * NWORDS

buf = bytearray()
buf += q.header(clk_div=q.CLK_DIV_8, mode=q.MODE_QUAD, flags=0)
buf += q.uart_tx(b"M4\n")
buf += q.delay_us(300000)
buf += q.uart_tx(f"H {NB}\n".encode())
buf += q.delay_us(50000)

# One 4-byte TLV per CS cycle.  Bytes are a simple incrementing
# pattern so we can see if DSP latches them verbatim or shifts.
for i in range(NWORDS):
    payload = bytes([4*i, 4*i+1, 4*i+2, 4*i+3])
    buf += q.mixed_xfer(b"", payload, 0)

buf += q.delay_us(500000)

with open("quad_4byte.qspi", "wb") as f:
    f.write(buf)
print(f"wrote quad_4byte.qspi ({len(buf)} bytes), {NWORDS} CS frames of 4 B")
