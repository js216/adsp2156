import os, random, sys
sys.path.insert(0, os.path.join(os.path.dirname(__file__),
                                "../../test_serv/examples"))
import make_qspi as q

def running_xor(d):
    a = 0
    for i in range(0, len(d), 4):
        a ^= (d[i]<<24)|(d[i+1]<<16)|(d[i+2]<<8)|d[i+3]
    return a & 0xFFFFFFFF

NB    = 1024 * 1024
# Push chunk size to the empirical ceiling.  65532 fails with
# FAILED_TO_WRITE_DEVICE; 65528 B/TLV (= 65524 user + 4 zero) is safe.
CHUNK = 65528
ZP    = b"\x00\x00\x00\x00"

random.seed(123)
data = bytes(random.getrandbits(8) for _ in range(NB))

# Split user data into chunks of (CHUNK - 4) bytes each.
user_per_tlv = CHUNK - len(ZP)   # 65524
chunks = []
for off in range(0, NB, user_per_tlv):
    chunks.append(data[off:off + user_per_tlv])
wire = b"".join(ZP + c for c in chunks)

print(f"payload = {NB} B  TLVs = {len(chunks)}  wire = {len(wire)} B")
print(f"expected cksum = 0x{running_xor(wire):08x}")

# Minimal delays: m4 command returns fast; just need a few ms for
# DSP to echo + process.  Final i also only needs a short window.
# All of the 1.8 s of padding in the safe version goes away.
buf = bytearray()
buf += q.header(clk_div=q.CLK_DIV_8, mode=q.MODE_QUAD, flags=0)
buf += q.uart_tx(b"m4\r");  buf += q.delay_us(20000)
# No reset `i` needed -- firmware boots with cksum=0 already.
for c in chunks:
    buf += q.mixed_xfer(b"", ZP + c, 0)
buf += q.delay_us(20000)
buf += q.uart_tx(b"i\r")
buf += q.delay_us(50000)

with open("quad_1mib_fast.qspi", "wb") as f: f.write(buf)
print("wrote quad_1mib_fast.qspi")
