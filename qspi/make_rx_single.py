import os, sys
sys.path.insert(0, os.path.join(os.path.dirname(__file__),
                                "../../test_serv/examples"))
import make_qspi as q

# Control test: DSP slave SINGLE RX + FT4222 master SingleWrite.
# Verifies the master actually drives CS+SCK in MODE_SINGLE on
# this board before we can make any claim about slave single TX.
# Expected DSP cksum for 8 words of 0xAA55AA55 = 0xaa55aa55 XOR'd 8
# times = 0x00000000.  Or pick a non-trivial payload.
PATTERN_WORD = 0xAA55AA55
N_WORDS = 8
PAYLOAD = (PATTERN_WORD).to_bytes(4, "big") * N_WORDS   # 32 bytes

def xor_words(b):
    a = 0
    for i in range(0, len(b), 4):
        a ^= (b[i]<<24)|(b[i+1]<<16)|(b[i+2]<<8)|b[i+3]
    return a & 0xFFFFFFFF
print(f"expected cksum = 0x{xor_words(PAYLOAD):08x}  (N_WORDS={N_WORDS})")

buf = bytearray()
buf += q.header(clk_div=q.CLK_DIV_8, mode=q.MODE_SINGLE, flags=0)
buf += q.uart_tx(b"?\r");   buf += q.delay_us(200000)
buf += q.uart_tx(b"a 0\r"); buf += q.delay_us(50000)
buf += q.mark("rx_start")
buf += q.xfer(PAYLOAD)   # SingleReadWrite -- clocks + reads MISO
buf += q.mark("rx_end")
buf += q.delay_us(50000)
buf += q.uart_tx(b"i\r")
buf += q.wait_uart(b"sum=0x", 2000)
with open("rx_single.qspi", "wb") as f:
    f.write(buf)
print("wrote rx_single.qspi")
