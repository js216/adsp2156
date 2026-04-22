import os, random, sys
sys.path.insert(0, os.path.join(os.path.dirname(__file__),
                                "../../test_serv/examples"))
import make_qspi as q

def running_xor(d):
    a = 0
    for i in range(0, len(d), 4):
        a ^= (d[i]<<24)|(d[i+1]<<16)|(d[i+2]<<8)|d[i+3]
    return a & 0xFFFFFFFF

NB    = 256 * 1024
CHUNK = 32768
ZP    = b"\x00\x00\x00\x00"

random.seed(123)
data = bytes(random.getrandbits(8) for _ in range(NB))

wire = b"".join(ZP + data[off:off+CHUNK] for off in range(0, NB, CHUNK))
print(f"payload = {NB} B  chunks = {NB // CHUNK}  wire = {len(wire)} B")
print(f"expected cksum = 0x{running_xor(wire):08x}")

buf = bytearray()
buf += q.header(clk_div=q.CLK_DIV_8, mode=q.MODE_QUAD, flags=0)
buf += q.uart_tx(b"m4\r");  buf += q.delay_us(300000)
buf += q.uart_tx(b"i\r");   buf += q.delay_us(200000)
for off in range(0, NB, CHUNK):
    buf += q.mixed_xfer(b"", ZP + data[off:off+CHUNK], 0)
buf += q.delay_us(800000)
buf += q.uart_tx(b"i\r")
buf += q.delay_us(200000)

with open("quad_256k_safe.qspi", "wb") as f: f.write(buf)
print("wrote quad_256k_safe.qspi")
