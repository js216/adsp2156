# SPDX-License-Identifier: GPL-3.0
# verify.py --- Per-check dispatcher for the sport demo
# Copyright (c) 2026 Jakob Kastelic

import os
import sys

sys.path.insert(0, os.path.join(
    os.path.dirname(os.path.abspath(__file__)), "..", "common"))
import verify_lib as v


UART_LOG = "streams/dsp.uart.bin"


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


def check_sweep_no_failures():
    text = _read_uart()
    sweep_lines = [line for line in text.splitlines() if "clkdiv=" in line]
    if not sweep_lines:
        sys.stderr.write("no 'clkdiv=' lines in uart log; sweep did not run\n")
        _dump_uart()
        return False
    for line in sweep_lines:
        if "fails=0 timeouts=0" not in line:
            sys.stderr.write(f"sweep line reported errors: {line}\n")
            _dump_uart()
            return False
    v._set_info(f"sweep_lines={len(sweep_lines)}")
    return True


DISPATCH = {
    "Check `test_serv` had no errors":
        v.check_no_errors,
    "Check fault LED is off":
        lambda: v.check_signal_inactive("DSP_FAULT"),
    "Check loopback sweep had no failures or timeouts":
        check_sweep_no_failures,
}


if __name__ == "__main__":
    sys.exit(v.main(DISPATCH, sys.argv))
