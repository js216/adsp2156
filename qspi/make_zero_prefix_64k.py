import os, random, sys
sys.path.insert(0, os.path.join(os.path.dirname(__file__),
                                "../../test_serv/examples"))
import make_qspi as q

def running_xor(d):
    a = 0
    for i in range(0, len(d), 4):
        a ^= (d[i]<<24)|(d[i+1]<<16)|(d[i+2]<<8)|d[i+3]
    return a & 0xFFFFFFFF

# seed-123 is the known-bad pattern.  63 KiB fails without prefix.
# Build a 63 KiB payload, prepend one zero word.  Total 64516 B
# which still fits in u16 (65535 max per MultiReadWrite).  If
# the prefix theory holds, DSP cksum should match.
PAYLOAD = 63 * 1024
random.seed(123)
data = bytes(random.getrandbits(8) for _ in range(PAYLOAD))

zero_prefix = b"\x00\x00\x00\x00"
combined = zero_prefix + data
print(f"total = {len(combined)} B  (prefix 4 B + payload {PAYLOAD} B)")
print(f"expected cksum (prefix+payload) = 0x{running_xor(combined):08x}")
print(f"prefix cksum                    = 0x{running_xor(zero_prefix):08x}")
print(f"payload-only cksum              = 0x{running_xor(data):08x}")

buf = bytearray()
buf += q.header(clk_div=q.CLK_DIV_8, mode=q.MODE_QUAD, flags=0)
buf += q.uart_tx(b"m4\r");  buf += q.delay_us(300000)
buf += q.uart_tx(b"i\r");   buf += q.delay_us(200000)
buf += q.mixed_xfer(b"", combined, 0)
buf += q.delay_us(300000)
buf += q.uart_tx(b"i\r")
buf += q.delay_us(200000)

with open("zero_prefix_64k.qspi", "wb") as f: f.write(buf)
print("wrote zero_prefix_64k.qspi")
