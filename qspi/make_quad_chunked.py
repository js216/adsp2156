# SPDX-License-Identifier: GPL-3.0
# make_quad_chunked.py --- 1 MiB quad via 16 B CS frames (byte-20 workaround)
# Copyright (c) 2026 Jakob Kastelic

# Quad slave RX corrupts after word 5 within any CS frame.  Chunk so
# each CS carries 4 words (16 B) -- safely below the onset -- and
# let DMA drain the RFIFO as the peripheral resets its shifter on
# every CS deassert.  DSP op_prbs_dma sees one continuous 1 MiB
# PRBS stream; byte errors should be zero.

import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__),
                                "../../test_serv/examples"))
import make_qspi as q

SEED  = 0x000DECAF
NBYTE = 1024 * 1024
CHUNK = 16  # 4 words per CS, well below the byte-20 fault threshold

buf = bytearray()
buf += q.header(clk_div=q.CLK_DIV_8, mode=q.MODE_QUAD, flags=0)
buf += q.uart_tx(b"M4\n")
buf += q.delay_us(400000)
buf += q.uart_tx(f"D {SEED:x} {NBYTE}\n".encode())
buf += q.delay_us(50000)

full = q.prbs_xorshift32(SEED, NBYTE)
for off in range(0, NBYTE, CHUNK):
    buf += q.mixed_xfer(b"", full[off:off + CHUNK], 0)

buf += q.delay_us(2000000)

with open("quad_chunked.qspi", "wb") as f:
    f.write(buf)
n_tlv = NBYTE // CHUNK
print(f"wrote quad_chunked.qspi ({len(buf)} bytes), {n_tlv} TLVs of {CHUNK} B")
