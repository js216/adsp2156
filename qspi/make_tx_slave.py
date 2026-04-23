import os, sys
sys.path.insert(0, os.path.join(os.path.dirname(__file__),
                                "../../test_serv/examples"))
import make_qspi as q

# DSP slave TX DMA test, single-lane first.  Sequence:
#   1. Boot, default SPI2 slave single-lane.
#   2. Startup-race filler so the direction flip below is clean.
#   3. `p <seed> <n>` stages PRBS + arms TX DMA (FLOW=STOP).
#   4. FT4222 master SingleRead pulls n bytes from MISO.
#   5. read_verify_prbs compares received bytes to host PRBS(seed,n).
SEED = 0xC0FFEE
N = 32       # small read so we can diff byte-by-byte from .bin

buf = bytearray()
buf += q.header(clk_div=q.CLK_DIV_8, mode=q.MODE_QUAD, flags=0)
# Put DSP in quad slave; then `q` stages polled TX on current lane width.
buf += q.uart_tx(b"m4\r");  buf += q.delay_us(50000)
buf += q.uart_tx(b"?\r");   buf += q.delay_us(200000)
buf += q.uart_tx(b"q\r");   buf += q.delay_us(50000)

buf += q.mark("polled_start")
# Master quad READ: single-write phase=0, multi-write phase=0,
# multi-read=16.  Master releases the data lanes during read phase
# so DSP slave can drive them.  Avoids the quad contention we hit
# with write-phase bytes.
buf += q.mixed_xfer(b"", b"", 16)
buf += q.mark("polled_end")
buf += q.delay_us(50000)
buf += q.uart_tx(b"x\r")

buf += q.delay_us(20000)
buf += q.wait_uart(b"tx ready", 2000)

with open("tx_slave.qspi", "wb") as f:
    f.write(buf)
print(f"wrote tx_slave.qspi seed=0x{SEED:08x} n={N}")
