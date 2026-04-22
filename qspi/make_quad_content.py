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

# Three 63 KiB payloads with different byte patterns.
SIZE = 63 * 1024
random.seed(123)
A = bytes(random.getrandbits(8) for _ in range(SIZE))
random.seed(999)
B = bytes(random.getrandbits(8) for _ in range(SIZE))
C = bytes([i & 0xFF for i in range(SIZE)])   # deterministic ramp

for tag, d in [("A", A), ("B", B), ("C", C)]:
    print(f"{tag}: 0x{running_xor(d):08x}")

for tag, d in [("A", A), ("B", B), ("C", C)]:
    buf = bytearray()
    buf += q.header(clk_div=q.CLK_DIV_8, mode=q.MODE_QUAD, flags=0)
    buf += q.uart_tx(b"m4\r");  buf += q.delay_us(300000)
    buf += q.uart_tx(b"i\r");   buf += q.delay_us(200000)
    buf += q.mixed_xfer(b"", d, 0)
    buf += q.delay_us(300000)
    buf += q.uart_tx(b"i\r")
    buf += q.delay_us(200000)
    fn = f"quad_content_{tag}.qspi"
    with open(fn, "wb") as f: f.write(buf)
    print(f"wrote {fn}")
