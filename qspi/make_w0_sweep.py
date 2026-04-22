import os, random, sys, struct
sys.path.insert(0, os.path.join(os.path.dirname(__file__),
                                "../../test_serv/examples"))
import make_qspi as q

random.seed(999); tail = bytes(random.getrandbits(8) for _ in range(20))  # 5 words

candidates = [
    0x00000022,
    0x22000000,
    0x02020202,
    0x20202020,
    0x22222222,
    0x00000000,
    0xffffffff,
    0x0d4416c4,  # s123 w0 -- known bad
    0xc8ad14e5,  # s999 w0 -- known good
]

buf = bytearray()
buf += q.header(clk_div=q.CLK_DIV_8, mode=q.MODE_QUAD, flags=0)
buf += q.uart_tx(b"m4\r");  buf += q.delay_us(300000)

print("--- expected ---")
for w0 in candidates:
    w0b = struct.pack(">I", w0)   # MSB-first byte packing
    payload = w0b + tail
    print(f"w0=0x{w0:08x}:")
    for i in range(0, len(payload), 4):
        w = (payload[i]<<24)|(payload[i+1]<<16)|(payload[i+2]<<8)|payload[i+3]
        print(f"  w{i//4}=0x{w:08x}")
    buf += q.uart_tx(f"--- {w0:08x} ---\r".encode())
    buf += q.delay_us(150000)
    buf += q.uart_tx(b"i\r");   buf += q.delay_us(150000)
    buf += q.mixed_xfer(b"", payload, 0)
    buf += q.delay_us(150000)
    buf += q.uart_tx(f"h {len(payload)}\r".encode())
    buf += q.delay_us(300000)

with open("w0_sweep.qspi", "wb") as f: f.write(buf)
print("\nwrote w0_sweep.qspi")
