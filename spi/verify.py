# SPDX-License-Identifier: GPL-3.0
# verify.py --- Per-check dispatcher for the spi demo
# Copyright (c) 2026 Jakob Kastelic

import os
import sys

sys.path.insert(0, os.path.join(
    os.path.dirname(os.path.abspath(__file__)), "..", "common"))
import verify_lib as v


UART_LOG = "streams/dsp.uart.bin"
BANNER = "spi pingpong dma+crc shell starting"
TEST_HDR = "test: master=SPI"
FAIL_MARKERS = ("WARN bit-error", "ERR ")


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


def check_banner():
    text = _read_uart()
    if BANNER not in text:
        sys.stderr.write(f"banner {BANNER!r} not in uart log\n")
        _dump_uart()
        return False
    return True


def check_test_invoked():
    text = _read_uart()
    if TEST_HDR not in text:
        sys.stderr.write(f"test header {TEST_HDR!r} not in uart log\n")
        _dump_uart()
        return False
    for m in FAIL_MARKERS:
        if m in text:
            sys.stderr.write(f"failure marker {m!r} in uart log\n")
            _dump_uart()
            return False
    return True


DISPATCH = {
    "Check `test_serv` had no errors":
        v.check_no_errors,
    "Check fault LED is off":
        lambda: v.check_signal_inactive("DSP_FAULT"),
    "Check SPI shell banner present":
        check_banner,
    "Check SPI test was invoked":
        check_test_invoked,
}


if __name__ == "__main__":
    sys.exit(v.main(DISPATCH, sys.argv))
