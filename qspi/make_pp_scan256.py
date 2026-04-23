import os, random, sys
sys.path.insert(0, os.path.join(os.path.dirname(__file__),
                                "../../test_serv/examples"))
import make_qspi as q

def running_xor(d):
    a = 0
    for i in range(0, len(d), 4):
        a ^= (d[i]<<24)|(d[i+1]<<16)|(d[i+2]<<8)|d[i+3]
    return a & 0xFFFFFFFF

# Scan every candidate first byte (0x00..0xFF) in FT4222 quad mode
# to see which values trigger the first-byte hazard (corrupted D1
# lane across the whole subsequent frame).  Each phase: send one
# 16-byte CS frame of (B, T, T, ..., T) where T is a fixed tail
# byte, then `i` to read the DSP's XOR checksum.  A match against
# the host's running_xor = clean; a mismatch = corruption.
#
# Payload is 16 bytes = 4 words so the 32-bit XOR accumulator is
# fed four distinct values, large enough to catch any byte-level
# drop or shift.
SEED = 123
FRAME_WORDS = 10        # 40 bytes total, word-aligned for DSP XOR.

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

# 39 bytes of PRBS, so [B, P0, P1, ..., P38] = 40 bytes = 10 words.
# No zero padding -- B sits adjacent to PRBS on the wire, exactly
# the arrangement that exposed the FT4222 first-byte hazard in the
# original 65528-byte saga.  Any zero between B and PRBS would
# re-apply the known prefix workaround and mask the fault.
PRBS_TAIL = _prbs(SEED, 4 * FRAME_WORDS - 1)

def build_frame(b):
    return bytes([b]) + PRBS_TAIL

# Print a host-side table of expected sums; user compares to the
# per-phase `sum=0x...` emitted over UART.
expected = {b: running_xor(build_frame(b)) for b in range(256)}
print("host-side expected cksums (hex):")
for b in range(0, 256, 16):
    row = " ".join(f"{b+i:02x}:{expected[b+i]:08x}" for i in range(16))
    print(row)

buf = bytearray()
buf += q.header(clk_div=q.CLK_DIV_8, mode=q.MODE_QUAD, flags=0)
# DSP to quad slave, plus the startup-race prime.
buf += q.uart_tx(b"m4\r");   buf += q.delay_us(50000)
buf += q.uart_tx(b"?\r");    buf += q.delay_us(200000)

buf += q.mark("scan_start")
for b in range(256):
    payload = build_frame(b)
    # Single CS frame per candidate.  First byte = b; rest = TAIL.
    buf += q.mixed_xfer(b"", payload, 0)
    # Drain + print sum, reset cksum for next phase.
    buf += q.delay_us(10000)
    buf += q.uart_tx(b"i\r")
    buf += q.delay_us(30000)
buf += q.mark("scan_end")

# wait_uart scans cumulative buffer, so any `sum=0x` hit -- the
# first phase's output is already in the buffer by the end of the
# scan.  This just flips early_done so runtime hold is skipped.
buf += q.wait_uart(b"sum=0x", 1000)

with open("pp_scan256.qspi", "wb") as f:
    f.write(buf)
print(f"\nwrote pp_scan256.qspi ({len(buf)} bytes, 256 phases)")
