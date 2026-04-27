# SPDX-License-Identifier: GPL-3.0
# verify.py --- Per-check dispatcher for the qspi demo
# Copyright (c) 2026 Jakob Kastelic

import os
import sys

sys.path.insert(0, os.path.join(
    os.path.dirname(os.path.abspath(__file__)), "..", "common"))
import verify_lib as v


UART_LOG = "streams/dsp.uart.bin"
BANNER = "qspi shell starting"
# Plan dumps 64 bytes = 16 words; last index is 0x0000000f.
LAST_WORD_TAG = "w0000000f="
MIN_WORDS = 16


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


def check_uart_output():
    text = _read_uart()
    if BANNER not in text:
        sys.stderr.write(f"banner {BANNER!r} not in uart log\n")
        _dump_uart()
        return False
    n_words = sum(1 for line in text.splitlines()
                  if line.lstrip().startswith("w") and "=0x" in line)
    v._set_info(f"{n_words} words")
    if LAST_WORD_TAG not in text:
        sys.stderr.write(
            f"last-word tag {LAST_WORD_TAG!r} not in uart log\n")
        _dump_uart()
        return False
    if n_words < MIN_WORDS:
        sys.stderr.write(
            f"too few hex-dump words: {n_words} < {MIN_WORDS}\n")
        _dump_uart()
        return False
    return True


DISPATCH = {
    "Check `test_serv` had no errors":
        v.check_no_errors,
    "Check fault LED is off":
        lambda: v.check_signal_inactive("DSP_FAULT"),
    "Check expected UART output present":
        check_uart_output,
}


if __name__ == "__main__":
    sys.exit(v.main(DISPATCH, sys.argv))
