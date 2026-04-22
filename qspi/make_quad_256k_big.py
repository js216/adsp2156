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
# Full chunks to the byte.  65528 is empirically the highest the
# FT4222 accepts (65532 already raises FAILED_TO_WRITE_DEVICE).
# Each wire TLV = 65528 B, first 4 B are zero (word-aligned so the
# DSP's 32-bit shifter sees a clean word-0 = 0x00000000, which is
# what defeats the FT4222 first-byte hazard and XORs to 0).
CHUNK = 65528
ZP    = b"\x00\x00\x00\x00"

random.seed(123)
data = bytes(random.getrandbits(8) for _ in range(NB))

user_per_tlv = CHUNK - len(ZP)   # 65524 bytes of user data per TLV
chunks = []
for off in range(0, NB, user_per_tlv):
    chunks.append(data[off:off + user_per_tlv])
wire = b"".join(ZP + c for c in chunks)
print(f"payload = {NB} B  TLVs = {len(chunks)}  wire = {len(wire)} B")
print(f"expected cksum = 0x{running_xor(wire):08x}")

buf = bytearray()
buf += q.header(clk_div=q.CLK_DIV_8, mode=q.MODE_QUAD, flags=0)
buf += q.uart_tx(b"m4\r");   buf += q.delay_us(50000)
buf += q.uart_tx(b"a 0\r");  buf += q.delay_us(50000)   # auto-consume off
buf += q.uart_tx(b"?\r");    buf += q.delay_us(50000)   # verify state
for c in chunks:
    buf += q.mixed_xfer(b"", ZP + c, 0)
buf += q.delay_us(200000)
buf += q.uart_tx(b"i\r")
buf += q.delay_us(50000)

with open("quad_256k_big.qspi", "wb") as f: f.write(buf)
print("wrote quad_256k_big.qspi")
