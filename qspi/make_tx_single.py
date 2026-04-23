import os, sys
sys.path.insert(0, os.path.join(os.path.dirname(__file__),
                                "../../test_serv/examples"))
import make_qspi as q

# Slave SINGLE TX polled bring-up.  Boot state is slave single RX;
# `q` switches to slave TX, pre-loads TFIFO with 4 known words,
# enables TEN.  FT4222 master SingleReadWrite clocks + reads MISO.
# Expected 16 MISO bytes:  aa55aa55 deadbeef cafebabe 12345678
buf = bytearray()
buf += q.header(clk_div=q.CLK_DIV_8, mode=q.MODE_SINGLE, flags=0)
buf += q.uart_tx(b"?\r");   buf += q.delay_us(200000)
# auto_consume=off so the idle-loop polling doesn't wreck the
# pre-q baseline xfer (rx_single / rx16.qspi need this too).
buf += q.uart_tx(b"a 0\r"); buf += q.delay_us(50000)
# Baseline xfer BEFORE any TX config: slave in RX, should clock.
buf += q.mark("pre_q_xfer")
buf += q.xfer(b"\xA5" * 16)
# Arm slave TX:
buf += q.uart_tx(b"q\r");   buf += q.delay_us(50000)
buf += q.mark("post_q_xfer")
buf += q.xfer(b"\x5A" * 16)
buf += q.delay_us(50000)
buf += q.uart_tx(b"x\r")
buf += q.delay_us(20000)
buf += q.uart_tx(b"i\r")
buf += q.wait_uart(b"sum=0x", 2000)
with open("tx_single.qspi", "wb") as f:
    f.write(buf)
print("wrote tx_single.qspi  expected: aa55aa55deadbeefcafebabe12345678")
