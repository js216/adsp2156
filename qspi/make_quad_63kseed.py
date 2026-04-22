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

random.seed(123)
a = bytes(random.getrandbits(8) for _ in range(63 * 1024))
print(f"0x{running_xor(a):08x}")

buf = bytearray()
buf += q.header(clk_div=q.CLK_DIV_8, mode=q.MODE_QUAD, flags=0)
buf += q.uart_tx(b"m4\r");  buf += q.delay_us(300000)
buf += q.uart_tx(b"i\r");   buf += q.delay_us(200000)
buf += q.mixed_xfer(b"", a, 0)
buf += q.delay_us(300000)
buf += q.uart_tx(b"i\r")
buf += q.delay_us(200000)

with open("quad_63kseed.qspi", "wb") as f: f.write(buf)
