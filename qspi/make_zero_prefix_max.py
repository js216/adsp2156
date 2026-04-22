import os, random, sys
sys.path.insert(0, os.path.join(os.path.dirname(__file__),
                                "../../test_serv/examples"))
import make_qspi as q

def running_xor(d):
    a = 0
    for i in range(0, len(d), 4):
        a ^= (d[i]<<24)|(d[i+1]<<16)|(d[i+2]<<8)|d[i+3]
    return a & 0xFFFFFFFF

# Max single-TLV nw is 65535 (u16).  To keep len multiple of 4 (DSP
# word boundary) use 65532 = 0xFFFC.  Prefix 4 B of zero.  Total
# transferred = 65536 B = 64 KiB exactly, fitting in one TLV.
PAYLOAD = 65532
random.seed(123)
data = bytes(random.getrandbits(8) for _ in range(PAYLOAD))
combined = b"\x00\x00\x00\x00" + data
# Have to truncate by 4 to fit in u16 nw=65535... actually 65536 > 65535.
# Trim payload by one word to land at 65532 total.
if len(combined) > 65532:
    combined = combined[:65532]
    # now combined = 4 zero + 65528 data
    data = combined[4:]

print(f"total = {len(combined)} B (= 0x{len(combined):x})")
print(f"expected cksum = 0x{running_xor(combined):08x}")
print(f"payload-only cksum = 0x{running_xor(data):08x}")

buf = bytearray()
buf += q.header(clk_div=q.CLK_DIV_8, mode=q.MODE_QUAD, flags=0)
buf += q.uart_tx(b"m4\r");  buf += q.delay_us(300000)
buf += q.uart_tx(b"i\r");   buf += q.delay_us(200000)
buf += q.mixed_xfer(b"", combined, 0)
buf += q.delay_us(300000)
buf += q.uart_tx(b"i\r")
buf += q.delay_us(200000)

with open("zero_prefix_max.qspi", "wb") as f: f.write(buf)
print("wrote zero_prefix_max.qspi")
