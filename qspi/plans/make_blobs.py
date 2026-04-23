# SPDX-License-Identifier: MIT
# Generates input blobs for the qspi/plans/*.txt test plans.
# Each function here produces ONE binary file consumed as @name
# by the new-format test_serv submit.py.
#
# Usage:
#   cd qspi && python3 plans/make_blobs.py
# Produces everything under qspi/blobs/.

import os
import random
import struct
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.abspath(os.path.join(HERE, "..", "blobs"))
os.makedirs(OUT, exist_ok=True)


def _write(name, data):
    p = os.path.join(OUT, name)
    with open(p, "wb") as f:
        f.write(data)
    print(f"  {name:<28}  {len(data):>10} B")


def running_xor(d):
    a = 0
    for i in range(0, len(d), 4):
        a ^= (d[i] << 24) | (d[i + 1] << 16) | (d[i + 2] << 8) | d[i + 3]
    return a & 0xFFFFFFFF


def prbs_xorshift32(seed, n):
    x = seed & 0xFFFFFFFF or 1
    out = bytearray(n)
    for i in range(n):
        x ^= (x << 13) & 0xFFFFFFFF
        x ^= (x >> 17) & 0xFFFFFFFF
        x ^= (x << 5) & 0xFFFFFFFF
        out[i] = x & 0xFF
    return bytes(out)


# ------------------------------------------------------------------
# pp_halves: 1 / 2 / 4 half-buffer worth of wire data.  Each half =
# 256 KiB.  Wire = sequence of 65528-byte chunks each prefixed with
# a 4-byte zero word (FT4222 first-byte-hazard workaround).  Total
# wire length is exactly N * 262144 bytes so DSP's per-half drain
# aligns byte-for-byte with the expected running_xor.

HALF = 256 * 1024
CHUNK = 65528
ZP = b"\x00\x00\x00\x00"


def build_pp_halves(n_halves):
    total = n_halves * HALF
    random.seed(123)
    n_full = total // CHUNK
    tail = total - n_full * CHUNK
    user_full = CHUNK - len(ZP)
    user_tail = tail - len(ZP) if tail else 0
    user_bytes = n_full * user_full + user_tail
    data = bytes(random.getrandbits(8) for _ in range(user_bytes))
    chunks = []
    off = 0
    for _ in range(n_full):
        chunks.append(data[off:off + user_full])
        off += user_full
    if tail:
        chunks.append(data[off:off + user_tail])
    wire = b"".join(ZP + c for c in chunks)
    assert len(wire) == total, (len(wire), total)
    return wire, running_xor(wire)


for n_h, tag in [(1, "1h"), (2, "2h"), (4, "4h")]:
    wire, cksum = build_pp_halves(n_h)
    _write(f"pp_{tag}_wire.bin", wire)
    print(f"    -> expected sum = 0x{cksum:08x}  ({n_h} halves)")


# ------------------------------------------------------------------
# pp_sweep: one big blob reused across every (mode, div) phase.  Use
# 2 halves so each phase completes predictably and the per-phase
# checksum is stable.

wire2, _ = build_pp_halves(2)
_write("pp_sweep_data.bin", wire2)


# ------------------------------------------------------------------
# rx_single control: 32 bytes = 8 words of 0xAA55AA55.  XOR of 8
# identical words is zero, so DSP cksum should print sum=0x00000000.

_write("rx_single_data.bin", (0xAA55AA55).to_bytes(4, "big") * 8)


# ------------------------------------------------------------------
# First-byte scan: one 10 240-byte blob of 256 consecutive 40-byte
# frames.  Frame i = [i] + PRBS(seed=123)[0..38].  Post-processing
# has to read the DSP's cumulative sum= and check against the XOR
# of the whole blob; per-B breakdown requires a separate run.

SEED = 123
N_PER = 40
frames = bytearray()
for b in range(256):
    frames.append(b)
    frames.extend(prbs_xorshift32(SEED, N_PER - 1))
_write("scan_frames.bin", bytes(frames))
print(f"    -> expected sum = 0x{running_xor(bytes(frames)):08x}  (all 256)")


print(f"\nwrote blobs under {OUT}/")
