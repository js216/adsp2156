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

# 40-byte frame = 10 words.  Tail is PRBS seed=123.  For each
# byte-position P in 0..3, vary that byte over 0..255 and keep the
# rest = PRBS-123.  Position 0 was already scanned; include here
# again as control so all four positions use the same phase cadence.
SEED = 123
FRAME_LEN = 40
PRBS_FULL = _prbs(SEED, FRAME_LEN)

POSITIONS = [0, 1, 2, 3]

buf = bytearray()
buf += q.header(clk_div=q.CLK_DIV_8, mode=q.MODE_QUAD, flags=0)
buf += q.uart_tx(b"m4\r");  buf += q.delay_us(50000)
buf += q.uart_tx(b"?\r");   buf += q.delay_us(200000)

for pos in POSITIONS:
    buf += q.mark(f"pos{pos}_start")
    for b in range(256):
        frame = bytearray(PRBS_FULL)
        frame[pos] = b
        buf += q.mixed_xfer(b"", bytes(frame), 0)
        buf += q.delay_us(5000)
        buf += q.uart_tx(b"i\r")
        buf += q.delay_us(15000)
    buf += q.mark(f"pos{pos}_end")

buf += q.wait_uart(b"sum=0x", 1000)

with open("pp_scan_pos.qspi", "wb") as f:
    f.write(buf)
print(f"wrote pp_scan_pos.qspi ({len(buf)} bytes, "
      f"{len(POSITIONS)*256} phases)")
print(f"PRBS seed={SEED} first 8 bytes = {PRBS_FULL[:8].hex()}")
