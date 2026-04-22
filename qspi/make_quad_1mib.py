# SPDX-License-Identifier: GPL-3.0
# make_quad_1mib.py --- Quad 1 MiB across multiple CS cycles via poller
# Copyright (c) 2026 Jakob Kastelic

# Sends 1 MiB quad-lane as 32 x 32 KiB TLVs through the poller's
# mixed_xfer handler (= spiMaster_MultiReadWrite(b"", chunk, 0) per
# call).  Each TLV is its own CS cycle.  DSP reports final checksum
# via the new shell's `i` command; compare against Python XOR.

import os, random, struct, sys
sys.path.insert(0, os.path.join(os.path.dirname(__file__),
                                "../../test_serv/examples"))
import make_qspi as q

SEED    = 0xC0FFEE12
NB      = 1024 * 1024
CHUNK   = 32768   # u16-safe, matches FT4222 MultiReadWrite cap

def running_xor(data):
    acc = 0
    for i in range(0, len(data), 4):
        w = (data[i]<<24)|(data[i+1]<<16)|(data[i+2]<<8)|data[i+3]
        acc ^= w
    return acc & 0xFFFFFFFF

# Fixed-seed random so the Python checksum is reproducible in UART
# output.  random.seed + getrandbits gives us a deterministic stream.
random.seed(SEED)
data = bytes(random.getrandbits(8) for _ in range(NB))
print(f"expected sum = 0x{running_xor(data):08x}")

buf = bytearray()
buf += q.header(clk_div=q.CLK_DIV_8, mode=q.MODE_QUAD, flags=0)
buf += q.uart_tx(b"m4\r")
buf += q.delay_us(400000)
buf += q.uart_tx(b"i\r")          # reset DSP cksum
buf += q.delay_us(200000)

for off in range(0, NB, CHUNK):
    buf += q.mixed_xfer(b"", data[off:off + CHUNK], 0)

buf += q.delay_us(500000)
buf += q.uart_tx(b"i\r")          # DSP prints cksum

with open("quad_1mib.qspi", "wb") as f:
    f.write(buf)
print(f"wrote quad_1mib.qspi ({len(buf)} bytes), {NB // CHUNK} TLVs")
