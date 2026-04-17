#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
# SPDX-FileCopyrightText: 2026 Jakob Kastelic
#
# Generate a compile_commands.json for clang-tidy and cppcheck.
#
# The real cross-compiler (cc21k) uses non-standard flags that
# clang cannot parse, so this script reads the top-level Makefile
# to discover demo directories, then emits clang-compatible
# entries for every .c file in the source tree.

import json
import re
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
COMMON = ROOT / "common"
BOARD = ROOT / "board"

# Freestanding headers provided by our tree (excluded from analysis)
SHIM_HEADERS = {"stdint.h", "stdarg.h", "stdbool.h", "stdio.h"}

FLAGS = (
    "-std=c99 -ffreestanding -Wno-everything "
    f"-I{COMMON} -I{BOARD} "
    f"-include {COMMON}/stdint.h "
    f"-include {COMMON}/stdbool.h "
    "-D__ADSPSHARC__=0x200 -D_LANGUAGE_C"
)


def read_demos():
    """Parse DEMOS from the top-level Makefile."""
    makefile = ROOT / "Makefile"
    text = makefile.read_text()
    # Match 'DEMOS = blink uart ...' possibly continued with backslash
    m = re.search(r"^DEMOS\s*[?:]?=\s*(.+?)(?:\n(?!\s)|\Z)", text,
                  re.MULTILINE | re.DOTALL)
    if not m:
        print("warning: DEMOS not found in Makefile, scanning all dirs",
              file=sys.stderr)
        return [d for d in ROOT.iterdir()
                if d.is_dir() and (d / "Makefile").exists()
                and d.name not in ("common", "board", "build", "scripts")]
    raw = m.group(1).replace("\\", " ")
    return [ROOT / name for name in raw.split()]


def find_sources():
    dirs = [COMMON, BOARD] + read_demos()
    sources = []
    for d in dirs:
        if d.is_dir():
            sources.extend(d.glob("*.c"))
    return [s for s in sources if s.name not in SHIM_HEADERS]


def main():
    srcs = find_sources()
    entries = [
        {
            "directory": str(src.parent),
            "command": f"clang {FLAGS} -c {src.name}",
            "file": str(src),
        }
        for src in sorted(srcs)
    ]
    out = ROOT / "build" / "compile_commands.json"
    out.parent.mkdir(exist_ok=True)
    with open(out, "w") as f:
        json.dump(entries, f, indent=2)
    print(f"wrote {out} ({len(entries)} entries)")


if __name__ == "__main__":
    main()
