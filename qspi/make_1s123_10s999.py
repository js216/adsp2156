import os, random, sys
sys.path.insert(0, os.path.join(os.path.dirname(__file__),
                                "../../test_serv/examples"))
import make_qspi as q

random.seed(123); s123 = bytes(random.getrandbits(8) for _ in range(4))   # 1 word
random.seed(999); s999 = bytes(random.getrandbits(8) for _ in range(40))  # 10 words

combined = s123 + s999
print(f"len = {len(combined)} B / {len(combined)//4} words")
for i in range(0, len(combined), 4):
    w = (combined[i]<<24)|(combined[i+1]<<16)|(combined[i+2]<<8)|combined[i+3]
    tag = "s123" if i < 4 else "s999"
    print(f"  w{i//4:2d} {tag}  expected=0x{w:08x}")

buf = bytearray()
buf += q.header(clk_div=q.CLK_DIV_8, mode=q.MODE_QUAD, flags=0)
buf += q.uart_tx(b"m4\r");  buf += q.delay_us(300000)
buf += q.uart_tx(b"i\r");   buf += q.delay_us(200000)
buf += q.mixed_xfer(b"", combined, 0)
buf += q.delay_us(200000)
buf += q.uart_tx(b"h 44\r");  buf += q.delay_us(500000)

with open("s123_then_s999.qspi", "wb") as f: f.write(buf)
print("wrote s123_then_s999.qspi")
