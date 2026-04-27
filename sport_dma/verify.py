# SPDX-License-Identifier: GPL-3.0
# verify.py --- Per-check dispatcher for the sport_dma demo
# Copyright (c) 2026 Jakob Kastelic

import os
import re
import sys

sys.path.insert(0, os.path.join(
    os.path.dirname(os.path.abspath(__file__)), "..", "common"))
import verify_lib as v


UART_LOG = "streams/dsp.uart.bin"
PASS_RE = re.compile(
    r"clkdiv=(\d+)\s+(\d+)\s+Hz\s+PASS\s+bytes=(\d+)")
FAIL_RE = re.compile(
    r"clkdiv=\d+\s+\d+\s+Hz\s+FAIL")
MISMATCH_RE = re.compile(r"^r=\d+\s+seed=[0-9a-fA-F]+\s+mm=(\d+)",
                         re.MULTILINE)


def _read_uart():
    with open(UART_LOG, "rb") as f:
        return f.read().decode("ascii", errors="replace")


def _dump_uart():
    text = _read_uart()
    sys.stderr.write("--- uart log ---\n")
    sys.stderr.write(text)
    if not text.endswith("\n"):
        sys.stderr.write("\n")
    sys.stderr.write("--- end uart log ---\n")


def check_loopback_pass():
    text = _read_uart()
    if FAIL_RE.search(text):
        sys.stderr.write("FAIL marker present in uart log\n")
        _dump_uart()
        return False
    bad_rounds = [m.group(1) for m in MISMATCH_RE.finditer(text)
                  if int(m.group(1)) != 0]
    if bad_rounds:
        sys.stderr.write(
            f"per-round mismatches present: {bad_rounds}\n")
        _dump_uart()
        return False
    m = PASS_RE.search(text)
    if not m:
        sys.stderr.write("no PASS marker in uart log\n")
        _dump_uart()
        return False
    v._set_info(
        f"clkdiv={m.group(1)} {m.group(2)} Hz bytes={m.group(3)}")
    return True


DISPATCH = {
    "Check `test_serv` had no errors":
        v.check_no_errors,
    "Check fault LED is off":
        lambda: v.check_signal_inactive("DSP_FAULT"),
    "Check loopback passes":
        check_loopback_pass,
}


if __name__ == "__main__":
    sys.exit(v.main(DISPATCH, sys.argv))
