import os, sys
sys.path.insert(0, os.path.join(os.path.dirname(__file__),
                                "../../test_serv/examples"))
import make_qspi as q

# DSP slave TX in DUAL-lane mode.  FT4222 master in DUAL.  Master
# drives CS+SCK and does a multi-read phase; slave shifts TFIFO
# onto D0/D1 (PA1/PA0).
buf = bytearray()
buf += q.header(clk_div=q.CLK_DIV_8, mode=q.MODE_DUAL, flags=0)
buf += q.uart_tx(b"m2\r");  buf += q.delay_us(50000)
buf += q.uart_tx(b"?\r");   buf += q.delay_us(200000)
buf += q.uart_tx(b"q\r");   buf += q.delay_us(50000)
buf += q.mark("xfer_start")
buf += q.mixed_xfer(b"", b"", 16)   # master releases lanes, reads 16B
buf += q.mark("xfer_end")
buf += q.delay_us(50000)
buf += q.uart_tx(b"x\r")
buf += q.delay_us(20000)
buf += q.wait_uart(b"tx_stat", 2000)
with open("tx_dual.qspi", "wb") as f:
    f.write(buf)
print("wrote tx_dual.qspi  expected: aa55aa55deadbeefcafebabe12345678")
