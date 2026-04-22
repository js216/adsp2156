import os, random, sys
sys.path.insert(0, os.path.join(os.path.dirname(__file__),
                                "../../test_serv/examples"))
import make_qspi as q

def running_xor(d):
    a = 0
    for i in range(0, len(d), 4):
        a ^= (d[i]<<24)|(d[i+1]<<16)|(d[i+2]<<8)|d[i+3]
    return a & 0xFFFFFFFF

# Ping-pong validation: 1 MiB over quad mode.  Goal is to prove that
# the two-descriptor DSCL flow handles bursts > ring size without the
# data-loss the old FLOW=AUTO + XCNT_CUR poller suffered above 256 KiB.
NB    = 1024 * 1024
CHUNK = 65528
ZP    = b"\x00\x00\x00\x00"

random.seed(123)
data = bytes(random.getrandbits(8) for _ in range(NB))

user_per_tlv = CHUNK - len(ZP)
chunks = []
for off in range(0, NB, user_per_tlv):
    chunks.append(data[off:off + user_per_tlv])
wire = b"".join(ZP + c for c in chunks)

print(f"payload = {NB} B  TLVs = {len(chunks)}  wire = {len(wire)} B")
print(f"expected cksum = 0x{running_xor(wire):08x}")

buf = bytearray()
buf += q.header(clk_div=q.CLK_DIV_8, mode=q.MODE_QUAD, flags=0)
buf += q.uart_tx(b"m4\r");  buf += q.delay_us(30000)
for c in chunks:
    buf += q.mixed_xfer(b"", ZP + c, 0)
buf += q.delay_us(100000)
buf += q.uart_tx(b"i\r")
buf += q.delay_us(50000)

with open("pp_1mib.qspi", "wb") as f:
    f.write(buf)
print("wrote pp_1mib.qspi")
