#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
# SPDX-FileCopyrightText: 2026 Jakob Kastelic
#
# Generate a compile_commands.json for clang-tidy from our
# source tree. Since the real cross-compiler (cc21k) uses
# non-standard flags that clang cannot parse, this script
# emits clang-compatible entries that let clang-tidy analyse
# the code for style and correctness patterns.

import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
COMMON = ROOT / "common"
BOARD  = ROOT / "board"

FLAGS = "-std=c99 -ffreestanding -Wno-everything " \
        f"-I{COMMON} -I{BOARD} " \
        f"-include {COMMON}/stdint.h " \
        f"-include {COMMON}/stdbool.h " \
        "-D__ADSPSHARC__=0x200 -D_LANGUAGE_C"

def find_sources():
    dirs = [COMMON, BOARD]
    for d in ["blink", "uart", "gpio", "sport", "sport_dma"]:
        dirs.append(ROOT / d)
    sources = []
    for d in dirs:
        sources.extend(d.glob("*.[ch]"))
    return [s for s in sources if s.name not in
            ("stdint.h", "stdarg.h", "stdbool.h", "stdio.h")]

def main():
    entries = []
    for src in find_sources():
        if src.suffix == ".h":
            continue
        entries.append({
            "directory": str(src.parent),
            "command": f"clang {FLAGS} -c {src.name}",
            "file": str(src),
        })
    out = ROOT / "build" / "compile_commands.json"
    out.parent.mkdir(exist_ok=True)
    with open(out, "w") as f:
        json.dump(entries, f, indent=2)
    print(f"wrote {out} ({len(entries)} entries)")

if __name__ == "__main__":
    main()
