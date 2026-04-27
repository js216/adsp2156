#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
# SPDX-FileCopyrightText: 2026 Jakob Kastelic
#
# Generate a per-example compile_commands.json for clang-tidy and
# cppcheck. The real cross-compiler (cc21k) uses non-standard flags
# that clang cannot parse, so this script emits clang-compatible
# entries for every .c file in the example dir plus common/ and the
# libsel printf used by stdio.

import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
COMMON = ROOT / "common"
LIBC = ROOT.parent / "selache" / "libsel"
LIBC_INC = LIBC / "include"
LIBC_STDIO = LIBC / "src" / "stdio"

FLAGS = (
    "-std=c99 -ffreestanding -Wno-everything "
    f"-I{COMMON} -I{LIBC_INC} "
    "-D__ADSPSHARC__=0x200 -D_LANGUAGE_C"
)


def main():
    if len(sys.argv) != 2:
        print(f"Usage: {sys.argv[0]} EXAMPLE_DIR", file=sys.stderr)
        sys.exit(1)
    example = Path(sys.argv[1]).resolve()
    if not example.is_dir():
        print(f"not a directory: {example}", file=sys.stderr)
        sys.exit(1)

    sources = sorted(list(COMMON.glob("*.c")) + list(example.glob("*.c")))
    # Pull libsel's printf.c into the analysis scope so whole-program
    # checks see it as the caller of putchar(); the other libsel stdio
    # sources are declared in the header but intentionally unused in
    # this tree and would show up as false-positive unusedFunction hits.
    libc_printf = LIBC_STDIO / "printf.c"
    if libc_printf.is_file():
        sources.append(libc_printf)

    entries = [
        {
            "directory": str(src.parent),
            "command": f"clang {FLAGS} -c {src.name}",
            "file": str(src),
        }
        for src in sources
    ]
    out_dir = example / "build"
    out_dir.mkdir(exist_ok=True)
    out = out_dir / "compile_commands.json"
    with open(out, "w") as f:
        json.dump(entries, f, indent=2)
    print(f"wrote {out} ({len(entries)} entries)")


if __name__ == "__main__":
    main()
