# SPDX-License-Identifier: GPL-3.0
# make_step7.py --- Step 7 probe: DSP master V (RX+verify PRBS)
# Copyright (c) 2026 Jakob Kastelic

# The DSP runs as SPI master (command V = clock bytes with
# MOSI=0 and verify MISO against xorshift32). The FT4222 runs
# as slave and must have the PRBS payload queued in its TX
# FIFO before the DSP starts clocking, so the order is:
#
#   1. init FT4222 slave (v2 header role=SLAVE)
#   2. uart_tx "M1\n"          - DSP in single-lane
#   3. uart_tx "Rm 9\n"        - DSP becomes master, 9.375 MHz
#   4. slave_write_prbs(...)   - queue PRBS into FT4222 slave FIFO
#   5. uart_tx "V decaf 64\n"  - DSP clocks bytes, verifies MISO
#   6. uart_tx "Rs\n"          - DSP back to slave so the tail
#                                of the job doesn't fight the
#                                FT4222's idle lines

import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__),
                                "../../test_serv/examples"))
import make_qspi as q

buf = bytearray()
buf += q.header_v2(role=q.ROLE_SLAVE, sys_clk=q.SYS_CLK_80,
                   clk_div=q.CLK_DIV_8, mode=q.MODE_SINGLE, flags=0)
buf += q.uart_tx(b"M1\n")
buf += q.delay_us(500000)
buf += q.uart_tx(b"Rm 9\n")
buf += q.delay_us(500000)
# Prime the FT4222 slave's TX FIFO before the DSP starts clocking.
buf += q.slave_write_prbs(0x000DECAF, 64)
buf += q.delay_us(50000)
buf += q.uart_tx(b"V decaf 64\n")
buf += q.delay_us(500000)
buf += q.uart_tx(b"Rs\n")
buf += q.delay_us(200000)

with open("step7.qspi", "wb") as f:
    f.write(buf)
print(f"wrote step7.qspi ({len(buf)} bytes)")
