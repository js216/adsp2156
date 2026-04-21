#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
# SPDX-FileCopyrightText: 2026 Jakob Kastelic
#
# Generate a compile_commands.json for clang-tidy and cppcheck.
#
# The real cross-compiler (cc21k) uses non-standard flags that
# clang cannot parse, so this script reads the top-level Makefile
# to discover demo directories, then emits clang-compatible
# entries for every .c file in the source tree. Stdlib headers
# come from the libsel tree alongside the project.

import json
import re
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
COMMON = ROOT / "common"
LIBSEL = ROOT.parent / "selache" / "libsel"
LIBSEL_INC = LIBSEL / "include"
LIBSEL_STDIO = LIBSEL / "src" / "stdio"

FLAGS = (
    "-std=c99 -ffreestanding -Wno-everything "
    f"-I{COMMON} -I{LIBSEL_INC} "
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
                and d.name not in ("common", "build", "scripts")]
    raw = m.group(1).replace("\\", " ")
    return [ROOT / name for name in raw.split()]


def find_sources():
    dirs = [COMMON] + read_demos()
    sources = []
    for d in dirs:
        if d.is_dir():
            sources.extend(d.glob("*.c"))
    # Also pull libsel's printf.c into the analysis scope so
    # whole-program checks see it as the caller of putchar();
    # the other libsel stdio sources (snprintf, sprintf, ...)
    # are declared in the header but intentionally unused in
    # this tree and would show up as false-positive
    # unusedFunction hits if included.
    libsel_printf = LIBSEL_STDIO / "printf.c"
    if libsel_printf.is_file():
        sources.append(libsel_printf)
    return sources


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
