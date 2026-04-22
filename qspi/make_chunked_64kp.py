import os, random, sys
sys.path.insert(0, os.path.join(os.path.dirname(__file__),
                                "../../test_serv/examples"))
import make_qspi as q

def running_xor(d):
    a = 0
    for i in range(0, len(d), 4):
        a ^= (d[i]<<24)|(d[i+1]<<16)|(d[i+2]<<8)|d[i+3]
    return a & 0xFFFFFFFF

# 64 KiB + 4 B -- just over cap, forces multiple TLVs.
random.seed(123)
data = bytes(random.getrandbits(8) for _ in range(64 * 1024 + 4))

# Chunk using the Jupyter pattern but at 32 KiB (safely under cap,
# and leaves room for a 4 B zero prefix per chunk => 32772 B per TLV).
CHUNK = 32768
ZP    = b"\x00\x00\x00\x00"

# What DSP will actually see on the wire, and thus what the DSP's
# running XOR accumulates.  Host computes the same over the exact
# bytes sent (including prefixes) so we can compare apples-to-apples.
wire = b""
for off in range(0, len(data), CHUNK):
    wire += ZP + data[off:off+CHUNK]

print(f"data bytes   = {len(data)}  (= 64 KiB + 4)")
print(f"chunks       = {(len(data)+CHUNK-1)//CHUNK}")
print(f"wire bytes   = {len(wire)}  (incl. prefixes)")
print(f"expected sum over wire bytes = 0x{running_xor(wire):08x}")

buf = bytearray()
buf += q.header(clk_div=q.CLK_DIV_8, mode=q.MODE_QUAD, flags=0)
buf += q.uart_tx(b"m4\r");  buf += q.delay_us(300000)
buf += q.uart_tx(b"i\r");   buf += q.delay_us(200000)
for off in range(0, len(data), CHUNK):
    payload = ZP + data[off:off+CHUNK]
    buf += q.mixed_xfer(b"", payload, 0)
buf += q.delay_us(500000)
buf += q.uart_tx(b"i\r")
buf += q.delay_us(200000)

with open("chunked_64kp.qspi", "wb") as f: f.write(buf)
print("wrote chunked_64kp.qspi")
