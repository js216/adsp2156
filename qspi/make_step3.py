# SPDX-License-Identifier: GPL-3.0
# make_step3.py --- Build step3.qspi for TODO.md step 3 (lane sweep)
# Copyright (c) 2026 Jakob Kastelic

# Combined job: exercises the four (lane-width x polled/DMA) variants
# requested by Step 3:
#
#   1. M2 + polled (`P c0ffee 64`)   dual-lane, polled RX
#   2. M2 + DMA    (`D c0ffee 64`)   dual-lane, SPI2_RX DMA
#   3. M4 + polled (`P c0ffee 64`)   quad-lane, polled RX
#   4. M4 + DMA    (`D c0ffee 64`)   quad-lane, SPI2_RX DMA
#
# For each the host clocks the PRBS stream out through the FT4222
# master's multi-IO lanes (D0/D1 in dual, D0..D3 in quad) by way of
# ``make_qspi.write_prbs_multi``, which wraps ``mixed_xfer`` with an
# empty single-IO phase and no read.  The ordinary ``write_prbs``
# path uses ``spiMaster_SingleWrite``, which drives MOSI only even
# when the FT4222 is initialised in DUAL/QUAD mode and therefore
# cannot exercise the DSP's dual/quad slave receive.
#
# State persistence in the DSP command loop lets us issue
# ``M2 -> P -> D -> M4 -> P -> D`` in one session without re-booting;
# the FT4222 master is switched between DUAL and QUAD with a
# ``reinit`` op before the M4 block.

import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__),
                                "../../test_serv/examples"))
import make_qspi as q

# Command-loop settle times.  Matches the 500 ms / 50 ms pair used by
# step1.qspi and step2.qspi: 500 ms after the UART command lets the
# DSP print its echo + state changes, 50 ms after the arming command
# lets the RX path enter the polling / DMA wait before the first SCLK
# edge arrives.
DELAY_CMD_US = 500000
DELAY_ARM_US = 50000

buf = bytearray()
# v1 header: start in DUAL mode at DIV_8, flags=0
# (CPOL idle-low, CPHA clock-trailing = CPHA=1 standard).
buf += q.header(clk_div=q.CLK_DIV_8, mode=q.MODE_DUAL, flags=0)

# ---- M2 (dual) polled -----------------------------------------------
buf += q.uart_tx(b"M2\n")
buf += q.delay_us(DELAY_CMD_US)
buf += q.uart_tx(b"P c0ffee 64\n")
buf += q.delay_us(DELAY_ARM_US)
buf += q.write_prbs_multi(0x00C0FFEE, 64)
buf += q.delay_us(DELAY_CMD_US)

# ---- M2 (dual) DMA --------------------------------------------------
buf += q.uart_tx(b"D c0ffee 64\n")
buf += q.delay_us(DELAY_ARM_US)
buf += q.write_prbs_multi(0x00C0FFEE, 64)
buf += q.delay_us(DELAY_CMD_US)

# ---- Switch FT4222 master to QUAD -----------------------------------
buf += q.reinit(q.CLK_DIV_8, q.MODE_QUAD, 0)

# ---- M4 (quad) polled ----------------------------------------------
buf += q.uart_tx(b"M4\n")
buf += q.delay_us(DELAY_CMD_US)
buf += q.uart_tx(b"P c0ffee 64\n")
buf += q.delay_us(DELAY_ARM_US)
buf += q.write_prbs_multi(0x00C0FFEE, 64)
buf += q.delay_us(DELAY_CMD_US)

# ---- M4 (quad) DMA --------------------------------------------------
buf += q.uart_tx(b"D c0ffee 64\n")
buf += q.delay_us(DELAY_ARM_US)
buf += q.write_prbs_multi(0x00C0FFEE, 64)
buf += q.delay_us(DELAY_CMD_US)

with open("step3.qspi", "wb") as f:
    f.write(buf)
print(f"wrote step3.qspi ({len(buf)} bytes)")
