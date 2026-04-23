import os, sys
sys.path.insert(0, os.path.join(os.path.dirname(__file__),
                                "../../test_serv/examples"))
import make_qspi as q

def _prbs(seed, n):
    x = seed & 0xFFFFFFFF
    if x == 0: x = 1
    out = bytearray(n)
    for i in range(n):
        x ^= (x << 13) & 0xFFFFFFFF
        x ^= (x >> 17) & 0xFFFFFFFF
        x ^= (x << 5)  & 0xFFFFFFFF
        out[i] = x & 0xFF
    return bytes(out)

def xor_words(buf):
    a = 0
    for i in range(0, len(buf), 4):
        a ^= (buf[i]<<24)|(buf[i+1]<<16)|(buf[i+2]<<8)|buf[i+3]
    return a & 0xFFFFFFFF

N_BYTES = 40   # 10 words
SEED = 123
frame = _prbs(SEED, N_BYTES)
expected = xor_words(frame)
print(f"frame = PRBS seed={SEED} len={N_BYTES}")
print(f"first 8 bytes = {frame[:8].hex()}")
print(f"expected cksum = 0x{expected:08x}")

buf = bytearray()
buf += q.header(clk_div=q.CLK_DIV_8, mode=q.MODE_QUAD, flags=0)
buf += q.uart_tx(b"m4\r");  buf += q.delay_us(50000)
buf += q.uart_tx(b"?\r");   buf += q.delay_us(200000)

# Send the raw PRBS frame 8 times so we can see if result is
# stable vs intermittent.
for i in range(8):
    buf += q.mixed_xfer(b"", frame, 0)
    buf += q.delay_us(10000)
    buf += q.uart_tx(b"i\r")
    buf += q.delay_us(30000)

buf += q.wait_uart(b"sum=0x", 1000)

with open("pp_raw_prbs.qspi", "wb") as f:
    f.write(buf)
print(f"wrote pp_raw_prbs.qspi ({len(buf)} bytes)")
