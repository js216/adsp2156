# SPDX-License-Identifier: GPL-3.0
# verify.py --- Per-check dispatcher for the uart demo
# Copyright (c) 2026 Jakob Kastelic

import os
import sys

sys.path.insert(0, os.path.join(
    os.path.dirname(os.path.abspath(__file__)), "..", "common"))
import verify_lib as v


UART_LOG = "streams/dsp.uart.bin"
BANNER = "uart demo starting"
# Plan delays 5000 ms; firmware ticks every 300 ms. Allow startup slack.
MIN_TICKS = 12


def _read_uart():
    with open(UART_LOG, "rb") as f:
        return f.read().decode("ascii", errors="replace")


def _dump_uart():
    sys.stderr.write("--- uart log ---\n")
    sys.stderr.write(_read_uart())
    if not _read_uart().endswith("\n"):
        sys.stderr.write("\n")
    sys.stderr.write("--- end uart log ---\n")


def check_uart_banner():
    text = _read_uart()
    if BANNER not in text:
        sys.stderr.write(f"banner {BANNER!r} not in uart log\n")
        _dump_uart()
        return False
    return True


def _parse_ticks():
    nums = []
    for line in _read_uart().splitlines():
        s = line.strip()
        if not s:
            continue
        try:
            nums.append(int(s, 16))
        except ValueError:
            continue
    return nums


def check_uart_count():
    nums = _parse_ticks()
    v._set_info(f"{len(nums)} ticks")
    if len(nums) < MIN_TICKS:
        sys.stderr.write(
            f"too few ticks: {len(nums)} < {MIN_TICKS}\n")
        _dump_uart()
        return False
    return True


def check_uart_sequential():
    nums = _parse_ticks()
    v._set_info(f"{len(nums)} ticks")
    if len(nums) < 2:
        sys.stderr.write(
            f"need >= 2 ticks to check ordering, got {len(nums)}\n")
        return False
    for i in range(1, len(nums)):
        if nums[i] - nums[i - 1] != 1:
            sys.stderr.write(
                f"non-sequential at index {i}: "
                f"{nums[i-1]:x} -> {nums[i]:x}\n")
            _dump_uart()
            return False
    return True


DISPATCH = {
    "Check `test_serv` had no errors":
        v.check_no_errors,
    "Check fault LED is off":
        lambda: v.check_signal_inactive("DSP_FAULT"),
    "Check UART banner present":
        check_uart_banner,
    "Check enough ticks have been printed":
        check_uart_count,
    "Check UART tick numbers are sequential":
        check_uart_sequential,
}


if __name__ == "__main__":
    sys.exit(v.main(DISPATCH, sys.argv))
