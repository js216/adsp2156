# SPDX-License-Identifier: GPL-3.0
# verify.py --- Per-check dispatcher for the fir demo
# Copyright (c) 2026 Jakob Kastelic

import os
import sys

sys.path.insert(0, os.path.join(
    os.path.dirname(os.path.abspath(__file__)), "..", "common"))
import verify_lib as v


UART_LOG = "streams/dsp.uart.bin"
BANNER = "fir demo starting"
DONE = "fir demo done"
PASS_MARKER = "PASS lpf_attenuation"
FAIL_MARKER = "FAIL lpf_attenuation"


def _read_uart():
    with open(UART_LOG, "rb") as f:
        return f.read().decode("ascii", errors="replace")


def _dump_uart():
    sys.stderr.write("--- uart log ---\n")
    text = _read_uart()
    sys.stderr.write(text)
    if not text.endswith("\n"):
        sys.stderr.write("\n")
    sys.stderr.write("--- end uart log ---\n")


def check_fir_output_present():
    text = _read_uart()
    if not text:
        sys.stderr.write("uart log is empty\n")
        return False
    missing = [m for m in (BANNER, "in_peak ", "ss_peak ", DONE)
               if m not in text]
    if missing:
        sys.stderr.write(f"missing markers: {missing}\n")
        _dump_uart()
        return False
    v._set_info(f"{len(text)} bytes")
    return True


def check_fir_attenuation():
    text = _read_uart()
    if FAIL_MARKER in text:
        sys.stderr.write(f"firmware reported {FAIL_MARKER!r}\n")
        _dump_uart()
        return False
    if PASS_MARKER not in text:
        sys.stderr.write(f"missing {PASS_MARKER!r} in uart log\n")
        _dump_uart()
        return False
    return True


DISPATCH = {
    "Check `test_serv` had no errors":
        v.check_no_errors,
    "Check fault LED is off":
        lambda: v.check_signal_inactive("DSP_FAULT"),
    "Check FIR output present":
        check_fir_output_present,
    "Check FIR attenuation passed":
        check_fir_attenuation,
}


if __name__ == "__main__":
    sys.exit(v.main(DISPATCH, sys.argv))
