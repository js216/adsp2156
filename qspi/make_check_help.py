import os, sys
sys.path.insert(0, os.path.join(os.path.dirname(__file__),
                                "../../test_serv/examples"))
import make_qspi as q

buf = bytearray()
buf += q.header(clk_div=q.CLK_DIV_8, mode=q.MODE_SINGLE, flags=0)
buf += q.uart_tx(b"help\r")
buf += q.delay_us(200000)

with open("check_help.qspi", "wb") as f: f.write(buf)
