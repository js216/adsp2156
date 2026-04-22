# SPDX-License-Identifier: GPL-3.0
# make_verify_edit.py --- Exercise backspace + up-arrow over UART
# Copyright (c) 2026 Jakob Kastelic

import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__),
                                "../../test_serv/examples"))
import make_qspi as q

buf = bytearray()
buf += q.header(clk_div=q.CLK_DIV_8, mode=q.MODE_SINGLE, flags=0)
buf += q.delay_us(400000)

# Type "BAD" then three DEL (0x7F) then "M1" then CR.  If backspace
# works, the committed line is "M1" and the shell answers "mode=x1".
# Otherwise the shell errors on "BADM1" or similar.
buf += q.uart_tx(b"BAD\x7f\x7f\x7fM1\r")
buf += q.delay_us(300000)

# Now send ESC [ A CR -- up-arrow + enter.  History should recall
# "M1" and run it again, printing "mode=x1" a second time.
buf += q.uart_tx(b"\x1b[A\r")
buf += q.delay_us(300000)

# Also test BS (0x08).
buf += q.uart_tx(b"ZZZM1\x08\x08\x08\x08\x08M1\r")
buf += q.delay_us(300000)

with open("verify_edit.qspi", "wb") as f:
    f.write(buf)
print("wrote verify_edit.qspi")
