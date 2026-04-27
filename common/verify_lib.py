# SPDX-License-Identifier: GPL-3.0
# verify_lib.py --- Shared check helpers and dispatcher harness
# Copyright (c) 2026 Jakob Kastelic

import csv
import json
import os
import sys


OUT_DIR = "test_out"
CHECKS_JSON = "checks.json"
SENTINEL = "sentinel.txt"
ERRORS = "errors.log"
SCOPE_CSV = "streams/scope.csv.bin"
SCOPE_SUMMARY = "streams/scope.summary.bin"

GREEN = "\033[32m"
RED = "\033[31m"
BOLD = "\033[1m"
RESET = "\033[0m"


_last_info = None


def _set_info(s):
    global _last_info
    _last_info = s


# --- artefact readers --------------------------------------------------

def read_scope_summary():
    with open(SCOPE_SUMMARY, "rb") as f:
        return json.loads(f.read())


def channel_summary(name):
    for row in read_scope_summary():
        if row.get("name") == name:
            return row
    return None


def read_scope_csv():
    """Return (t, {chan: [v, ...]}) parsed from scope CSV stream."""
    with open(SCOPE_CSV, "r", encoding="utf-8") as f:
        rdr = csv.reader(f)
        header = next(rdr)
        t = []
        cols = {h: [] for h in header[1:]}
        for row in rdr:
            t.append(float(row[0]))
            for i, h in enumerate(header[1:], 1):
                cols[h].append(float(row[i]))
    return t, cols


def rising_active_times(t, v, thresh):
    """Timestamps where v transitions from inactive (>= thresh) to
    active (< thresh)."""
    times = []
    prev = None
    for ts, val in zip(t, v):
        active = val < thresh
        if prev is False and active:
            times.append(ts)
        prev = active
    return times


# --- generic checks ----------------------------------------------------

def check_no_errors():
    if os.path.isfile(ERRORS):
        sys.stderr.write("errors.log present:\n")
        with open(ERRORS) as f:
            sys.stderr.write(f.read())
        return False
    if not os.path.isfile(SENTINEL):
        sys.stderr.write("sentinel.txt missing\n")
        return False
    with open(SENTINEL) as f:
        body = f.read()
    try:
        manifest = json.loads(body)
    except ValueError as e:
        sys.stderr.write(f"sentinel not JSON: {e}\n")
        return False
    n = manifest.get("n_errors")
    if n is None:
        sys.stderr.write("sentinel missing 'n_errors'\n")
        return False
    if n != 0:
        sys.stderr.write(f"sentinel reports n_errors={n}\n")
        return False
    return True


def check_signal_inactive(name):
    """Pass iff scope summary shows zero active-going edges for `name`."""
    row = channel_summary(name)
    if row is None:
        sys.stderr.write(f"{name} channel not in scope summary\n")
        return False
    n = row.get("went_active", 0)
    if n != 0:
        sys.stderr.write(f"{name} asserted: went_active={n}\n")
        return False
    return True


def check_signal_min_edges(name, minimum):
    row = channel_summary(name)
    if row is None:
        sys.stderr.write(f"{name} channel not in scope summary\n")
        return False
    n = row.get("went_active", 0)
    _set_info(f"went_active={n}")
    if n < minimum:
        sys.stderr.write(
            f"{name} too few edges: went_active={n} (need >= {minimum})\n")
        return False
    return True


def check_signal_period(channel, expected_ms, tol_ms, active_below):
    """Median rising-active period on `channel` must be within tolerance."""
    t, cols = read_scope_csv()
    if channel not in cols:
        sys.stderr.write(f"{channel} not in scope CSV\n")
        return False
    times = rising_active_times(t, cols[channel], active_below)
    if len(times) < 2:
        sys.stderr.write(
            f"{channel}: need >= 2 active transitions, got {len(times)}\n")
        return False
    deltas_ms = sorted((times[i + 1] - times[i]) * 1000.0
                       for i in range(len(times) - 1))
    median = deltas_ms[len(deltas_ms) // 2]
    _set_info(f"period={median:.1f} ms")
    if abs(median - expected_ms) > tol_ms:
        sys.stderr.write(
            f"{channel}: median period {median:.1f} ms outside "
            f"{expected_ms} +/- {tol_ms} ms\n")
        return False
    return True


# --- dispatcher harness ------------------------------------------------

def _run_one(dispatch, key):
    global _last_info
    _last_info = None
    fn = dispatch.get(key)
    if fn is None:
        sys.stderr.write(f"unknown check: {key!r}\n")
        return False
    try:
        return fn()
    except (OSError, ValueError, KeyError) as e:
        sys.stderr.write(f"error during check: {e}\n")
        return False


def _warn_unused(dispatch, used_keys):
    unused = sorted(set(dispatch.keys()) - used_keys)
    if unused:
        sys.stderr.write(
            f"{BOLD}WARNING{RESET}: DISPATCH keys defined but not "
            f"referenced by any README bullet:\n")
        for k in unused:
            sys.stderr.write(f"  {k}\n")


def _sweep(dispatch):
    checks_path = os.path.join(OUT_DIR, CHECKS_JSON)
    try:
        with open(checks_path) as f:
            checks_map = json.load(f)
    except OSError as e:
        sys.stderr.write(f"cannot read {checks_path}: {e}\n")
        return 1

    used = set()
    for bullets in checks_map.values():
        for b in bullets:
            used.add(b.strip())
    _warn_unused(dispatch, used)

    fails = 0
    total = 0
    cwd = os.getcwd()
    for h, bullets in checks_map.items():
        out_dir = os.path.join(cwd, OUT_DIR, h)
        print(f"\n=== block {h} ===")
        if not os.path.isdir(out_dir):
            print(f"{RED}{BOLD}MISSING{RESET}: {out_dir}")
            fails += len(bullets) or 1
            continue
        os.chdir(out_dir)
        try:
            block_ok = True
            for item in bullets:
                total += 1
                ok = _run_one(dispatch, item.strip())
                tag = (f"{GREEN}PASS{RESET}" if ok
                       else f"{RED}FAIL{RESET}")
                suffix = f" ({_last_info})" if _last_info else ""
                print(f"[{tag}] {item}{suffix}")
                if not ok:
                    fails += 1
                    block_ok = False
            banner = (f"{GREEN}{BOLD}SUCCESS{RESET}" if block_ok
                      else f"{RED}{BOLD}FAIL{RESET}")
            print(f"--- block {h}: {banner} ---")
        finally:
            os.chdir(cwd)

    word = "CHECK" if total == 1 else "CHECKS"
    summary = (f"{GREEN}{BOLD}{total} {word} PASSED{RESET}"
               if fails == 0
               else f"{RED}{BOLD}{fails}/{total} {word} FAILED{RESET}")
    print(f"\n{summary}")
    return 0 if fails == 0 else 1


def main(dispatch, argv):
    """Entry point. With no arg: sweep all hashes from checks.json.
    With one arg: dispatch a single check in CWD."""
    if len(argv) == 1:
        return _sweep(dispatch)
    if len(argv) != 2:
        sys.stderr.write("usage: verify.py [<key>]\n")
        return 1
    key = argv[1].strip()
    if key not in dispatch:
        sys.stderr.write(f"unknown check: {key!r}\n")
        sys.stderr.write("known keys:\n")
        for k in dispatch:
            sys.stderr.write(f"  {k}\n")
        return 1
    ok = _run_one(dispatch, key)
    if _last_info:
        sys.stdout.write(_last_info)
    return 0 if ok else 1
