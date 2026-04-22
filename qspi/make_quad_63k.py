# SPDX-License-Identifier: GPL-3.0
# make_quad_63k.py --- Exact poller replica of the Jupyter 63 KiB quad test
# Copyright (c) 2026 Jakob Kastelic

# Jupyter pattern:
#   spiMaster_Init(Mode.QUAD, CLK_DIV_8, ...)
#   setClock(CLK_80)
#   for off in range(0, 63*1024, 65536):
#       spiMaster_MultiReadWrite(b"", data[off:off+65536], 0)
#
# Replica: single mixed_xfer TLV with 63 KiB payload -> one
# spiMaster_MultiReadWrite call with ns=0, nw=64512, nr=0.
# Poller's _init_master already calls setClock(CLK_80) + quad init
# via the header TLV (clk_div=CLK_DIV_8, mode=MODE_QUAD, flags=0).

import os, random, sys
sys.path.insert(0, os.path.join(os.path.dirname(__file__),
                                "../../test_serv/examples"))
import make_qspi as q

def running_xor(data):
    acc = 0
    for i in range(0, len(data), 4):
        w = (data[i]<<24)|(data[i+1]<<16)|(data[i+2]<<8)|data[i+3]
        acc ^= w
    return acc & 0xFFFFFFFF

data = bytes(random.getrandbits(8) for _ in range(63 * 1024))
print(f"0x{running_xor(data):08x}")

buf = bytearray()
buf += q.header(clk_div=q.CLK_DIV_8, mode=q.MODE_QUAD, flags=0)
buf += q.uart_tx(b"m4\r");  buf += q.delay_us(400000)
buf += q.uart_tx(b"i\r");   buf += q.delay_us(200000)  # reset cksum
buf += q.mixed_xfer(b"", data, 0)                       # one call, 63 KiB
buf += q.delay_us(300000)
buf += q.uart_tx(b"i\r")                                # print cksum

with open("quad_63k.qspi", "wb") as f:
    f.write(buf)
print(f"wrote quad_63k.qspi ({len(buf)} bytes)")
