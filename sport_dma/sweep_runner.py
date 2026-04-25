# SPDX-License-Identifier: GPL-3.0
# sweep_runner.py --- Drive SPORT4 external clkdiv sweep with live per-bench progress
# Copyright (c) 2026 Jakob Kastelic

import os
import re
import shutil
import subprocess
import sys
import tempfile

SCLK_HZ = 93_750_000
HERE = os.path.dirname(os.path.abspath(__file__))
SUBMIT = os.path.abspath(os.path.join(HERE, "..", "..", "test_serv", "submit.py"))
PLAN = os.path.join(HERE, "plans", "single.txt")


def bit_hz(clkdiv):
    return SCLK_HZ // (2 * (clkdiv + 1))


def estimate_runtime_s(clkdiv):
    # The firmware's per-clkdiv dwell is capped at 3 s (DWELL_CEILING_US)
    # because the silicon traps after ~5 s of continuous SPORT operation
    # at sub-MHz bit rates.  Total wall time is one dwell plus startup,
    # boot, fill, verify, and printf overhead.
    return 8.0


def build(clkdiv, seed_hex):
    env = os.environ.copy()
    env["CFLAGS_EXTRA"] = (
        f"-DSWEEP_CLKDIV={clkdiv} -DLFSR_SEED_BASE=0x{seed_hex}U"
    )
    # Force a rebuild of main.doj (depends on SWEEP_CLKDIV macro) and the ldr.
    for f in ("main.doj", "main.dxe", "main.ldr"):
        try:
            os.remove(os.path.join(HERE, f))
        except FileNotFoundError:
            pass
    res = subprocess.run(["make"], cwd=HERE, env=env, capture_output=True, text=True)
    if res.returncode != 0:
        sys.stderr.write(res.stdout)
        sys.stderr.write(res.stderr)
        raise RuntimeError(f"build failed for clkdiv={clkdiv}")


def run_one(clkdiv, seed_hex="ACE1BEEF"):
    build(clkdiv, seed_hex)
    runtime = estimate_runtime_s(clkdiv)
    # Plan delay must cover the firmware sweep. Pad generously.
    delay_ms = int(runtime * 1000 * 2) + 10000
    plan_text = (
        "dsp:reset\n"
        "dsp:uart_open\n"
        "dsp:boot ldr=@main.ldr\n"
        f"delay ms={delay_ms}\n"
        "dsp:uart_close\n"
    )
    tmp_plan = tempfile.NamedTemporaryFile(
        mode="w", suffix=".txt", delete=False, dir=HERE
    )
    tmp_plan.write(plan_text)
    tmp_plan.close()
    extract = tempfile.mkdtemp(prefix=f"sport_clk{clkdiv}_")
    try:
        wait = delay_ms / 1000 + 120
        ldr = os.path.join(HERE, "main.ldr")
        cmd = [
            "python3", SUBMIT, tmp_plan.name,
            "--blob", f"main.ldr={ldr}",
            "--wait", str(wait),
            "--runtime", str(int(runtime + 8)),
            "--extract", extract,
        ]
        res = subprocess.run(cmd, capture_output=True, text=True)
        uart_path = os.path.join(extract, "streams", "dsp.uart.bin")
        if not os.path.exists(uart_path):
            sys.stderr.write(res.stdout)
            sys.stderr.write(res.stderr)
            return None, "no UART output"
        with open(uart_path, "rb") as f:
            uart = f.read().decode("utf-8", errors="replace")
        return uart, None
    finally:
        os.remove(tmp_plan.name)
        shutil.rmtree(extract, ignore_errors=True)


# Match firmware-emitted result line:
#   clkdiv=N H Hz PASS bytes=B rounds=R
#   clkdiv=N H Hz FAIL bytes=B rounds=R mm=M ...
RESULT_RE = re.compile(
    r"clkdiv=(\d+)\s+(\d+)\s+Hz\s+(PASS|FAIL)\s+bytes=(\d+)\s+rounds=(\d+)(?:\s+mm=(\d+))?"
)


def parse_result(uart, clkdiv):
    for line in uart.splitlines():
        m = RESULT_RE.search(line)
        if m and int(m.group(1)) == clkdiv:
            return {
                "clkdiv": int(m.group(1)),
                "rate_hz": int(m.group(2)),
                "status": m.group(3),
                "bytes": int(m.group(4)),
                "rounds": int(m.group(5)),
                "mismatches": int(m.group(6)) if m.group(6) else 0,
            }
    return None


def progress_line(clkdiv, result, err=None):
    rate = bit_hz(clkdiv)
    if result is None:
        return (f"[clkdiv={clkdiv} rate={rate} Hz] tested 0 bytes, "
                f"mismatches=?, status=ERROR ({err})")
    return (f"[clkdiv={result['clkdiv']} rate={result['rate_hz']} Hz] "
            f"tested {result['bytes']} bytes, "
            f"mismatches={result['mismatches']}, status={result['status']}")


SEEDS = ("ACE1BEEF", "5A5A1234")


def sweep(clkdivs):
    table = []
    for clk in clkdivs:
        total_bytes = 0
        total_mm    = 0
        status      = "PASS"
        rate        = bit_hz(clk)
        boots_ok    = 0
        for seed in SEEDS:
            uart, err = run_one(clk, seed_hex=seed)
            if err:
                status = "ERROR"
                break
            r = parse_result(uart, clk)
            if r is None:
                status = "ERROR"
                break
            total_bytes += r["bytes"]
            total_mm    += r["mismatches"]
            if r["status"] != "PASS":
                status = "FAIL"
            boots_ok += 1
        print(
            f"[clkdiv={clk} rate={rate} Hz] tested {total_bytes} bytes, "
            f"mismatches={total_mm}, status={status}",
            flush=True,
        )
        table.append((clk, total_bytes, total_mm, status))
    return table


if __name__ == "__main__":
    if len(sys.argv) > 1:
        divs = [int(x) for x in sys.argv[1:]]
    else:
        # Default boundary search: known fail (low clkdiv) -> known pass (high).
        divs = [91, 88, 80, 70, 60, 50, 40, 30, 23, 10, 5, 1]
    sweep(divs)
