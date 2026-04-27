# SPDX-License-Identifier: GPL-3.0
# verify.py --- Per-check dispatcher for the iir demo
# Copyright (c) 2026 Jakob Kastelic

import os
import re
import sys

sys.path.insert(0, os.path.join(
    os.path.dirname(os.path.abspath(__file__)), "..", "common"))
import verify_lib as v


UART_LOG = "streams/dsp.uart.bin"
BANNER = "iir demo starting"
DONE = "iir demo done"
PASS_LINE = "PASS iir_attenuation"
# Firmware prints PRINT_COUNT=8 steady-state samples as "y[<hex>] <hex>".
MIN_SAMPLES = 8
SAMPLE_RE = re.compile(r"^y\[[0-9a-fA-F]+\]\s+[0-9a-fA-F]+\s*$")


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


def check_iir_banner():
    text = _read_uart()
    if BANNER not in text:
        sys.stderr.write(f"banner {BANNER!r} not in uart log\n")
        _dump_uart()
        return False
    if DONE not in text:
        sys.stderr.write(f"done marker {DONE!r} not in uart log\n")
        _dump_uart()
        return False
    return True


def check_iir_samples():
    text = _read_uart()
    samples = [ln for ln in text.splitlines()
               if SAMPLE_RE.match(ln.strip())]
    v._set_info(f"{len(samples)} samples")
    if len(samples) < MIN_SAMPLES:
        sys.stderr.write(
            f"too few sample lines: {len(samples)} < {MIN_SAMPLES}\n")
        _dump_uart()
        return False
    return True


def check_iir_attenuation():
    text = _read_uart()
    if PASS_LINE not in text:
        sys.stderr.write(f"{PASS_LINE!r} not in uart log\n")
        _dump_uart()
        return False
    return True


DISPATCH = {
    "Check `test_serv` had no errors":
        v.check_no_errors,
    "Check fault LED is off":
        lambda: v.check_signal_inactive("DSP_FAULT"),
    "Check IIR banner present":
        check_iir_banner,
    "Check IIR output samples present":
        check_iir_samples,
    "Check IIR attenuation passed":
        check_iir_attenuation,
}


if __name__ == "__main__":
    sys.exit(v.main(DISPATCH, sys.argv))
