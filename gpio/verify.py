# SPDX-License-Identifier: GPL-3.0
# verify.py --- Per-check dispatcher for the gpio demo
# Copyright (c) 2026 Jakob Kastelic

import os
import sys

sys.path.insert(0, os.path.join(
    os.path.dirname(os.path.abspath(__file__)), "..", "common"))
import verify_lib as v


UART_LOG = "streams/dsp.uart.bin"
# The probe phase steals PA_06/PA_07 from UART0, so any byte the
# firmware emits during that phase is corrupted/lost. The scan
# header (`=== scan N ===`) is printed before the probe phase and is
# therefore mangled on the wire; the report lines printed after
# pinmux_uart0() restores the UART are clean. `connected:` is the
# first report line emitted by report_pairs() and is the strongest
# unambiguous marker that the scan finished and UART came back up.
REPORT_MARKER = "connected:"
HELP_MARKER = "commands:"
SCAN_DONE_MARKER = " done ==="
MIN_LOG_BYTES = 64
MIN_REPORT_LINES = 5
MIN_SCANS = 2


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


def check_uart_scan_present():
    text = _read_uart()
    v._set_info(f"{len(text)} bytes")
    if not text:
        sys.stderr.write("uart log is empty\n")
        return False
    if REPORT_MARKER not in text:
        sys.stderr.write(
            f"marker {REPORT_MARKER!r} not in uart log\n")
        _dump_uart()
        return False
    if len(text) < MIN_LOG_BYTES:
        sys.stderr.write(
            f"uart log too short: {len(text)} < {MIN_LOG_BYTES}\n")
        _dump_uart()
        return False
    return True


def check_scan_completed():
    """A `connected:` line can only be printed after the full probe
    sweep finishes and pinmux_uart0() restores the UART. Require
    several such lines to confirm report_pairs() ran to completion
    rather than producing a single straggler."""
    text = _read_uart()
    n = sum(1 for line in text.splitlines()
            if line.lstrip().startswith(REPORT_MARKER))
    v._set_info(f"{n} connected lines")
    if n < MIN_REPORT_LINES:
        sys.stderr.write(
            f"too few report lines: {n} < {MIN_REPORT_LINES}\n")
        _dump_uart()
        return False
    return True


def check_help_present():
    text = _read_uart()
    if HELP_MARKER not in text:
        sys.stderr.write(
            f"help marker {HELP_MARKER!r} not in uart log\n")
        _dump_uart()
        return False
    return True


def check_second_scan():
    """Count `=== scan N done ===` markers. The boot scan emits one;
    the `t` command kicks a second run_test that emits another. Two
    distinct done-markers prove the trigger command actually drove a
    second scan."""
    text = _read_uart()
    n = sum(1 for line in text.splitlines()
            if SCAN_DONE_MARKER in line)
    v._set_info(f"{n} scan done markers")
    if n < MIN_SCANS:
        sys.stderr.write(
            f"only {n} done markers (need >= {MIN_SCANS})\n")
        _dump_uart()
        return False
    return True


DISPATCH = {
    "Check `test_serv` had no errors":
        v.check_no_errors,
    "Check fault LED is off":
        lambda: v.check_signal_inactive("DSP_FAULT"),
    "Check UART scan output present":
        check_uart_scan_present,
    "Check scan completed":
        check_scan_completed,
    "Check help output present":
        check_help_present,
    "Check trigger command runs second scan":
        check_second_scan,
}


if __name__ == "__main__":
    sys.exit(v.main(DISPATCH, sys.argv))
