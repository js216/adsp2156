# SPDX-License-Identifier: GPL-3.0
# make_step4.py --- Build step4.qspi for TODO.md step 4 (slave x4 long burst)
# Copyright (c) 2026 Jakob Kastelic

# Step 4: sustained 1 MiB quad-lane slave RX as one continuous clocked
# multi-write from the FT4222 master. The DSP-side stress is the
# chunked DMA re-arm path, which in this series has been converted to
# a continuous FLOW=AUTO 64 KiB ring on DMA27; the CPU consumer
# chases XCNT_CUR and verifies words in place without ever stopping
# the channel (see qspi/main.c :: op_prbs_dma).
#
# Host-side framing. One mixed_xfer TLV per 32 KiB of payload -- the
# TLV header carries `nw` as a u16 so the 1 MiB stream is emitted as
# 32 TLVs. The poller feeds each TLV to one
# `spiMaster_MultiReadWrite(b"", mbuf, 0)` call; with sbuf empty the
# FT4222 clocks the full mbuf on the multi-IO lanes (D0..D3) in a
# single CS-framed burst. Step 4's whole point is "sustained
# clocking", so the host must not split CS between TLVs; the
# intra-TLV clocking is continuous.

import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__),
                                "../../test_serv/examples"))
import make_qspi as q

# 1 MiB PRBS burst (per Step 4 spec).
PRBS_SEED  = 0x000DECAF
PRBS_BYTES = 1024 * 1024

# TLV payload size. The mixed_xfer header encodes `nw` as a u16, so
# any single TLV is capped at 65535 B. 32768 B splits 1 MiB cleanly
# into 32 TLVs with comfortable u16 headroom.
TLV_PAYLOAD_BYTES = 32768

# Command-loop settle times (match step3.qspi).
DELAY_CMD_US = 500000
DELAY_ARM_US = 50000

# Trailing window. 1 MiB at DIV_8 quad is ~280 ms of pure SCLK plus
# USB overhead; 2 s leaves room for the DSP's final PRBSDMA status
# line to traverse UART before the capture window closes.
DELAY_TAIL_US = 2000000

assert PRBS_BYTES % TLV_PAYLOAD_BYTES == 0, \
    "PRBS_BYTES must be a whole multiple of TLV_PAYLOAD_BYTES"

buf = bytearray()
# v1 header: FT4222 master in QUAD mode at DIV_8, flags=0.
buf += q.header(clk_div=q.CLK_DIV_8, mode=q.MODE_QUAD, flags=0)

# ---- M4 (quad) DMA, 1 MiB ------------------------------------------
buf += q.uart_tx(b"M4\n")
buf += q.delay_us(DELAY_CMD_US)
buf += q.uart_tx(f"D decaf {PRBS_BYTES}\n".encode())
buf += q.delay_us(DELAY_ARM_US)

full = q.prbs_xorshift32(PRBS_SEED, PRBS_BYTES)
for off in range(0, PRBS_BYTES, TLV_PAYLOAD_BYTES):
    buf += q.mixed_xfer(b"", full[off:off + TLV_PAYLOAD_BYTES], 0)

buf += q.delay_us(DELAY_TAIL_US)

with open("step4.qspi", "wb") as f:
    f.write(buf)
print(f"wrote step4.qspi ({len(buf)} bytes), "
      f"{PRBS_BYTES // TLV_PAYLOAD_BYTES} TLVs")
