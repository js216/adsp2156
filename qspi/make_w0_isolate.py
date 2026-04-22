import os, random, sys, struct
sys.path.insert(0, os.path.join(os.path.dirname(__file__),
                                "../../test_serv/examples"))
import make_qspi as q

random.seed(999); tail = bytes(random.getrandbits(8) for _ in range(20))  # 5 words

# Sweep variants to isolate which byte / nibble / bit of 0x0d4416c4
# triggers the failure.  Tail is fixed s999 [0:5] so we can tell whether
# the receive path is clean after a given w0.
candidates = [
    # Baselines
    0x00000000,  # all zeros
    0x0d4416c4,  # known bad (s123 w0)
    0xc8ad14e5,  # known good (s999 w0)
    # Replace each byte of 0x0d4416c4 with 0x00
    0x004416c4,
    0x0d0016c4,
    0x0d440000 | 0x0000_00c4,   # = 0x0d4400c4
    0x0d441600,
    # First-byte variants
    0x004416c4,
    0x014416c4,
    0x024416c4,
    0x044416c4,
    0x084416c4,
    0x0c4416c4,
    0x0e4416c4,
    0x0f4416c4,
    0x1d4416c4,
    0x8d4416c4,
    0xfd4416c4,
    # Nibble-swap first byte
    0xd04416c4,
    # Flip each bit of byte 0 of 0x0d = 0000 1101
    0x0c4416c4,  # bit 0
    0x0f4416c4,  # bit 1 -- same as flip of bit
    0x094416c4,  # bit 2
    0x054416c4,  # bit 3
    0x1d4416c4,  # bit 4
    0x2d4416c4,  # bit 5
    0x4d4416c4,  # bit 6
    0x8d4416c4,  # bit 7
]

# Dedup while preserving order
seen = set()
cands = []
for c in candidates:
    if c not in seen:
        cands.append(c)
        seen.add(c)

buf = bytearray()
buf += q.header(clk_div=q.CLK_DIV_8, mode=q.MODE_QUAD, flags=0)
buf += q.uart_tx(b"m4\r");  buf += q.delay_us(300000)

print(f"{len(cands)} variants")
for w0 in cands:
    w0b = struct.pack(">I", w0)
    payload = w0b + tail
    # Tag in UART so output is easy to grep
    buf += q.uart_tx(f"# {w0:08x}\r".encode())
    buf += q.delay_us(120000)
    buf += q.uart_tx(b"i\r");  buf += q.delay_us(120000)
    buf += q.mixed_xfer(b"", payload, 0)
    buf += q.delay_us(120000)
    buf += q.uart_tx(f"h {len(payload)}\r".encode())
    buf += q.delay_us(250000)

with open("w0_isolate.qspi", "wb") as f: f.write(buf)
print("wrote w0_isolate.qspi")
