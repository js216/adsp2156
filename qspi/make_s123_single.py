import os, random, sys
sys.path.insert(0, os.path.join(os.path.dirname(__file__),
                                "../../test_serv/examples"))
import make_qspi as q

random.seed(123)
data = bytes(random.getrandbits(8) for _ in range(63 * 1024))

def running_xor(d):
    a = 0
    for i in range(0, len(d), 4):
        a ^= (d[i]<<24)|(d[i+1]<<16)|(d[i+2]<<8)|d[i+3]
    return a & 0xFFFFFFFF
print(f"expected 0x{running_xor(data):08x}")

buf = bytearray()
buf += q.header(clk_div=q.CLK_DIV_8, mode=q.MODE_SINGLE, flags=0)
buf += q.uart_tx(b"m1\r");  buf += q.delay_us(300000)
buf += q.uart_tx(b"i\r");   buf += q.delay_us(200000)
buf += q.write_prbs(0, 0)  # placeholder, won't use
# Actually send via write tag (tag 0x01) so single-lane SingleWrite.
# Rewrite: use tag 0x01 directly.
buf = bytearray()
buf += q.header(clk_div=q.CLK_DIV_8, mode=q.MODE_SINGLE, flags=0)
buf += q.uart_tx(b"m1\r");  buf += q.delay_us(300000)
buf += q.uart_tx(b"i\r");   buf += q.delay_us(200000)
buf += q.write(data)         # tag 0x01, chunked in poller
buf += q.delay_us(500000)
buf += q.uart_tx(b"i\r")

with open("s123_single.qspi", "wb") as f: f.write(buf)
print("wrote s123_single.qspi")
