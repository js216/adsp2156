# SPORT (unified FPGA test app)

The single parameterized SPORT test app that replaces the per-direction
DSP apps (`sport_fpga_bidir`, `sport_fpga_rx`, and the retired
`sport_fpga_tx`/`sport_fpga_2x`/`sport_fpga_4x`). One `main.c`, built per
direction from a `(RX_N, TX_N, flags)` point:

- `RX_N`   0..4  FPGA->DSP PRBS-31 lanes the DSP receives and verifies in
  DMA ping-pong halves (SPORT5B/1B/4B/0B).
- `TX_N`   0..4  DSP->FPGA lanes the DSP drives as clock/FS masters
  (SPORT4A/0A...); the FPGA receives and verifies these.
- `TX_NO_REFILL`  TX rings free-run as pure clock/FS masters (F->D-only rows).
- `TOTAL_WORDS`, `RX_HALF`/`TX_HALF`, `SPORT_SCLK_HZ`, `SPORT_CLKDIV`.

The report line is `sport_bidir rx_lanes=N tx_lanes=M rx_words=... rx_errors=...
e0..e3 ... slips=...` on the DSP UART; for D->F rows the FPGA reports its own
`sport_rx lanes=...` line (the data check is FPGA-side there).

Build selection: pass `-DTXONLY` (with `NCH`/`N_WORDS`/`HALF_WORDS`) for the
D->F transmitter engine; omit it (with `RX_N`/`TX_N`/`TOTAL_WORDS`) for the
RX/bidirectional engine. The Makefile picks the matching object link order.

Status: VERIFIED byte-faithful. Every per-direction build is byte-identical
(`.ldr` bin-diff) to the original app it replaces -- `-DTXONLY` builds match
`sport_fpga_rx` (NCH=1/2/4), non-`TXONLY` builds match `sport_fpga_bidir`
(1x1/2x2/4x). The merge therefore needs no bench re-prove; Phase 5 repoints
the missions here (adding `-DTXONLY` for the D->F rows) with no binary change.
