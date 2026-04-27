# SPDX-License-Identifier: GPL-3.0
# verify.py --- Per-check dispatcher for the blink demo
# Copyright (c) 2026 Jakob Kastelic

import os
import sys

sys.path.insert(0, os.path.join(
    os.path.dirname(os.path.abspath(__file__)), "..", "common"))
import verify_lib as v


# LED2 = LED4 = C1 toggles once per main.c PERIOD_MS = 900 ms.
EXPECTED_PERIOD_MS = 900.0
PERIOD_TOL_MS = 90.0
# Mirrors config.json scope.signals.C1.active_below.
ACTIVE_BELOW_C1 = 68.0
MIN_BLINKS = 2


DISPATCH = {
    "Check `test_serv` had no errors":
        v.check_no_errors,
    "Check fault LED is off":
        lambda: v.check_signal_inactive("DSP_FAULT"),
    "Check number of LED blinks is correct":
        lambda: v.check_signal_min_edges("DSP_LED", MIN_BLINKS),
    "Check LED blink spacing is correct":
        lambda: v.check_signal_period(
            "C1", EXPECTED_PERIOD_MS, PERIOD_TOL_MS, ACTIVE_BELOW_C1),
}


if __name__ == "__main__":
    sys.exit(v.main(DISPATCH, sys.argv))
