import os, random, sys
sys.path.insert(0, os.path.join(os.path.dirname(__file__),
                                "../../test_serv/examples"))
import make_qspi as q

def running_xor(d):
    a = 0
    for i in range(0, len(d), 4):
        a ^= (d[i]<<24)|(d[i+1]<<16)|(d[i+2]<<8)|d[i+3]
    return a & 0xFFFFFFFF

# Ping-pong halves are 256 KiB each; auto-consume drains N complete
# halves.  Size the wire exactly to N * 262144 B so host can verify
# level + sum with no trailing-partial-half ambiguity.
#
# Wire layout: (T-1) full TLVs of 65528 B  +  one tail TLV of R B.
# Each TLV = 4-byte zero prefix + user payload.
HALF = 256 * 1024
CHUNK = 65528
ZP = b"\x00\x00\x00\x00"

def build(n_halves, name):
    total = n_halves * HALF
    # How many full chunks fit.
    n_full = total // CHUNK
    tail = total - n_full * CHUNK  # bytes remaining for tail TLV
    assert tail >= len(ZP), f"tail too small: {tail}"

    random.seed(123)
    # Payload is total - (T * 4) user bytes where T = n_full + 1 TLVs.
    user_full = CHUNK - len(ZP)
    user_tail = tail - len(ZP)
    user_bytes = n_full * user_full + user_tail
    data = bytes(random.getrandbits(8) for _ in range(user_bytes))

    chunks = []
    off = 0
    for i in range(n_full):
        chunks.append(data[off:off + user_full]);  off += user_full
    if user_tail > 0:
        chunks.append(data[off:off + user_tail])
    wire = b"".join(ZP + c for c in chunks)
    assert len(wire) == total, f"wire={len(wire)} total={total}"

    print(f"{name}: halves={n_halves}  wire={len(wire)} B  TLVs={len(chunks)}"
          f"  expected cksum=0x{running_xor(wire):08x}")

    buf = bytearray()
    buf += q.header(clk_div=q.CLK_DIV_8, mode=q.MODE_QUAD, flags=0)
    buf += q.uart_tx(b"m4\r");  buf += q.delay_us(50000)
    # Script-level m4 -> rs -> m4 workaround for the lane-switch
    # startup race.  Driving straight into QUAD from boot drops
    # the opening burst; bouncing through SINGLE once primes the
    # hardware so the retry QUAD captures every byte.  Root cause
    # invisible in MMR state, see qspi_pp_startup_race memory.
    buf += q.uart_tx(b"rs\r");  buf += q.delay_us(50000)
    buf += q.uart_tx(b"m4\r");  buf += q.delay_us(50000)
    for c in chunks:
        buf += q.mixed_xfer(b"", ZP + c, 0)
    buf += q.delay_us(100000)
    buf += q.uart_tx(b"i\r")
    buf += q.delay_us(100000)

    with open(f"{name}.qspi", "wb") as f:
        f.write(buf)

for n, nm in [(1, "pp_1h"), (2, "pp_2h"), (4, "pp_4h")]:
    build(n, nm)
