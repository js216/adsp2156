# SPDX-License-Identifier: GPL-3.0
# make_quad_1byte.py --- Send 1 byte per CS via quad, dump raw words
# Copyright (c) 2026 Jakob Kastelic

# DSP is armed to receive 64 B via DMA (16 x 32-bit words), then
# the host clocks 64 single-byte TLVs on the quad lanes.  Each TLV
# raises CS low, drives 2 SCLK cycles (1 byte = 8 bits across 4
# lanes), raises CS high.  DSP's H command dumps every word in
# dma_rx_buf.  Expected finding: RFIFO only ingests on 32-bit word
# boundaries, so 1-byte CS frames never complete a word and DMA
# never advances -- H will hang waiting.

import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__),
                                "../../test_serv/examples"))
import make_qspi as q

NB = 64
buf = bytearray()
buf += q.header(clk_div=q.CLK_DIV_8, mode=q.MODE_QUAD, flags=0)
buf += q.uart_tx(b"M4\n")
buf += q.delay_us(300000)

# Arm DSP to receive NB bytes (16 words) and dump.
buf += q.uart_tx(f"H {NB}\n".encode())
buf += q.delay_us(50000)

# Send NB one-byte TLVs -- each its own CS frame on the quad lanes.
# Each byte of a known incrementing pattern so we can recognise it.
for i in range(NB):
    buf += q.mixed_xfer(b"", bytes([i & 0xFF]), 0)

buf += q.delay_us(500000)

with open("quad_1byte.qspi", "wb") as f:
    f.write(buf)
print(f"wrote quad_1byte.qspi ({len(buf)} bytes), NB={NB}")
