import os, random, sys
sys.path.insert(0, os.path.join(os.path.dirname(__file__),
                                "../../test_serv/examples"))
import make_qspi as q

def running_xor(d):
    a = 0
    for i in range(0, len(d), 4):
        a ^= (d[i]<<24)|(d[i+1]<<16)|(d[i+2]<<8)|d[i+3]
    return a & 0xFFFFFFFF

# Sweep all (lane width, SCK) combinations.  One .qspi file cycling
# mode m1/m2/m4 x FT4222 DIV_8/4/2 (= 10/20/40 MHz SCK at OpClk
# 80 MHz).  Each phase sends 1 MiB of wire; `i` prints per-phase
# cksum; wait_uart grabs the sum before advancing.
PAYLOAD = 1024 * 1024
CHUNK = 65528
ZP = b"\x00\x00\x00\x00"

random.seed(123)
# One shared data buffer used by all phases.  For SINGLE mode we
# stream it raw (no ZP framing); for DUAL/QUAD we chunk it into TLVs
# each prefixed with 4 zero bytes (first-byte hazard workaround).
data_single = bytes(random.getrandbits(8) for _ in range(PAYLOAD))

# Multi-lane wire: 17 TLVs of (ZP + up to 65524 user bytes).
random.seed(123)
n_full = PAYLOAD // CHUNK
tail = PAYLOAD - n_full * CHUNK
user_full = CHUNK - len(ZP)
user_tail = tail - len(ZP)
user_bytes = n_full * user_full + user_tail
data_multi_src = bytes(random.getrandbits(8) for _ in range(user_bytes))
chunks = []
off = 0
for _ in range(n_full):
    chunks.append(data_multi_src[off:off + user_full]); off += user_full
if user_tail > 0:
    chunks.append(data_multi_src[off:off + user_tail])
multi_wire = b"".join(ZP + c for c in chunks)
assert len(multi_wire) == PAYLOAD
# Single wire: just the raw PRNG stream, no framing.
single_wire = data_single
assert len(single_wire) == PAYLOAD

cksum_single = running_xor(single_wire)
cksum_multi = running_xor(multi_wire)
print(f"single-lane cksum = 0x{cksum_single:08x}")
print(f"multi-lane  cksum = 0x{cksum_multi:08x}")

MODES = [
    (1, "m1", q.MODE_SINGLE, False),   # False => use q.write
    (2, "m2", q.MODE_DUAL,   True),    # True  => use q.mixed_xfer
    (4, "m4", q.MODE_QUAD,   True),
]
DIVS = [
    (q.CLK_DIV_8, "div8", 10),   # 10 MHz SCK
    (q.CLK_DIV_4, "div4", 20),   # 20 MHz SCK
    (q.CLK_DIV_2, "div2", 40),   # 40 MHz SCK
]

buf = bytearray()
buf += q.header(clk_div=q.CLK_DIV_8, mode=q.MODE_SINGLE, flags=0)

for mnum, mname, modecode, multi in MODES:
    # DSP-side lane change + startup-race prime.
    buf += q.uart_tx(f"m{mnum}\r".encode());  buf += q.delay_us(50000)
    buf += q.uart_tx(b"?\r");                 buf += q.delay_us(200000)

    for divcode, dname, sckmhz in DIVS:
        phase = f"{mname}_{dname}_{sckmhz}MHz"
        # Retune FT4222 to this (mode, divisor).  Re-issuing reinit
        # every phase covers the clock change and keeps the
        # per-call lane-width consistent with the DSP side.
        buf += q.reinit(divcode, modecode, 0)
        buf += q.delay_us(10000)

        buf += q.mark(f"{phase}_start")
        if multi:
            for c in chunks:
                buf += q.mixed_xfer(b"", ZP + c, 0)
        else:
            buf += q.write(single_wire)
        buf += q.mark(f"{phase}_end")

        buf += q.delay_us(50000)
        buf += q.uart_tx(b"i\r")
        # Each `i` resets the DSP cksum, so this sum is per-phase.
        buf += q.wait_uart(b"sum=0x", 3000)

with open("pp_sweep.qspi", "wb") as f:
    f.write(buf)
print(f"wrote pp_sweep.qspi ({len(buf)} bytes, "
      f"{len(MODES)*len(DIVS)} phases)")
