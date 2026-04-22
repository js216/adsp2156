import os, random, sys
sys.path.insert(0, os.path.join(os.path.dirname(__file__),
                                "../../test_serv/examples"))
import make_qspi as q

random.seed(123);  s123 = bytes(random.getrandbits(8) for _ in range(40))

print(f"len = {len(s123)} B / {len(s123)//4} words")
for i in range(0, len(s123), 4):
    w = (s123[i]<<24)|(s123[i+1]<<16)|(s123[i+2]<<8)|s123[i+3]
    print(f"  w{i//4:2d}  expected=0x{w:08x}")

buf = bytearray()
buf += q.header(clk_div=q.CLK_DIV_8, mode=q.MODE_QUAD, flags=0)
buf += q.uart_tx(b"m4\r");  buf += q.delay_us(300000)
buf += q.uart_tx(b"i\r");   buf += q.delay_us(200000)
buf += q.mixed_xfer(b"", s123, 0)
buf += q.delay_us(200000)
buf += q.uart_tx(b"h 40\r");  buf += q.delay_us(500000)

with open("s123_alone.qspi", "wb") as f: f.write(buf)
print("wrote s123_alone.qspi")
