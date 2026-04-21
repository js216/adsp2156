#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
# SPDX-FileCopyrightText: 2026 Jakob Kastelic
"""
Generate a Graphviz inclusion graph for C/C++ projects.

Each source file is grouped by its parent directory. Files in
`common/` form a shared pool; every other directory that owns a
main.c is treated as a separate demo. One `digraph` block is
emitted per demo, showing only the inclusion closure reachable
from that demo's main.c. Feeding the output to `dot -Tpdf`
produces a multi-page PDF with one page per demo.

Features:
- Merges .c/.cpp with the corresponding .h/.hpp into a single
  node (both have the same Path.stem).
- Resolves quoted #include directives by exact relative path,
  falling back to basename match within the active file set.
- Detects inclusion cycles per demo and exits 1 if any exist.
- Writes DOT to stdout.
"""

import sys
import re
from pathlib import Path
from collections import defaultdict, deque

INCLUDE_RE = re.compile(r'^\s*#\s*include\s*"([^"]+)"')

COMMON_DIR = "common"


def parse_includes(file_path):
    """Return list of quoted #include filenames in the file."""
    includes = []
    try:
        with open(file_path, 'r', encoding='utf-8', errors='ignore') as f:
            for line in f:
                m = INCLUDE_RE.match(line)
                if m:
                    includes.append(m.group(1))
    except OSError as e:
        print(f"Warning: failed to read {file_path}: {e}", file=sys.stderr)
    return includes


def node_name(path, parent_dir):
    """Node name; demo main.c files are tagged with their parent
    directory so they do not collide with each other."""
    stem = path.stem
    if stem == "main":
        return f"{parent_dir}/main"
    return stem


def group_files(files):
    """Split argv file list into (common_files, demos). `demos`
    is a dict {demo_name: [paths]} for every non-common directory
    that contains at least one file."""
    common = []
    demos = defaultdict(list)
    for raw in files:
        p = Path(raw)
        parent = p.parent.name or "."
        if parent == COMMON_DIR:
            common.append(p)
        else:
            demos[parent].append(p)
    return common, dict(demos)


def build_edges(files):
    """Full inclusion graph over `files` only. Returns a dict
    {src_node -> set(tgt_node)} plus a map {node -> path}."""
    files = [Path(f) for f in files]
    path_map = {str(f): f for f in files}
    basename_map = {f.name: f for f in files}
    node_of = {}
    for f in files:
        node_of[f] = node_name(f, f.parent.name or ".")

    graph = defaultdict(set)
    for f in files:
        src = node_of[f]
        for inc in parse_includes(f):
            tgt_path = None
            # exact path match (relative to cwd)
            for candidate in (inc, f"{f.parent}/{inc}",
                              f"{COMMON_DIR}/{Path(inc).name}"):
                if candidate in path_map:
                    tgt_path = path_map[candidate]
                    break
            if tgt_path is None and Path(inc).name in basename_map:
                tgt_path = basename_map[Path(inc).name]
            if tgt_path is None:
                continue
            tgt = node_of[tgt_path]
            if tgt != src:
                graph[src].add(tgt)
    return graph


def reachable(graph, root):
    """BFS from `root` over `graph`. Returns the set of nodes
    reachable from root (inclusive)."""
    seen = {root}
    q = deque([root])
    while q:
        n = q.popleft()
        for m in graph.get(n, ()):
            if m not in seen:
                seen.add(m)
                q.append(m)
    return seen


def detect_cycles(graph, nodes):
    """DFS over the subgraph induced by `nodes`. Returns list of
    cycles (each cycle is a list of nodes)."""
    visited = set()
    stack = set()
    cycles = []

    def visit(node, path):
        if node in stack:
            cycle_start = path.index(node)
            cycles.append(path[cycle_start:])
            return
        if node in visited:
            return
        visited.add(node)
        stack.add(node)
        for neighbor in graph.get(node, []):
            if neighbor in nodes:
                visit(neighbor, path + [neighbor])
        stack.remove(node)

    for n in nodes:
        visit(n, [n])
    return cycles


def format_graphviz(name, graph, nodes, cycles):
    """Return a DOT string for the induced subgraph of `graph`
    restricted to `nodes`."""
    lines = [
        f"digraph {name} {{",
        "  node [shape=box];",
    ]
    cycle_edges = set()
    for c in cycles:
        for i in range(len(c)):
            src = c[i]
            tgt = c[(i + 1) % len(c)]
            cycle_edges.add((src, tgt))

    for src in sorted(nodes):
        for tgt in sorted(graph.get(src, ())):
            if tgt not in nodes:
                continue
            if (src, tgt) in cycle_edges:
                lines.append(f'  "{src}" -> "{tgt}" [color=red];')
            else:
                lines.append(f'  "{src}" -> "{tgt}";')
    lines.append("}")
    return "\n".join(lines) + "\n"


def main():
    args = sys.argv[1:]
    outdir = None
    if len(args) >= 2 and args[0] == "-o":
        outdir = Path(args[1])
        args = args[2:]
    if not args:
        print(f"Usage: {sys.argv[0]} [-o OUTDIR] <file1> <file2> ...",
              file=sys.stderr)
        sys.exit(1)

    common, demos = group_files(args)

    if not demos:
        print("No demo directories found (nothing outside common/).",
              file=sys.stderr)
        sys.exit(1)

    if outdir is not None:
        outdir.mkdir(parents=True, exist_ok=True)

    exit_code = 0
    for demo in sorted(demos):
        active = common + demos[demo]
        graph = build_edges(active)
        main_path = next((p for p in demos[demo] if p.name == "main.c"), None)
        if main_path is None:
            print(f"Warning: no main.c in {demo}/, skipping", file=sys.stderr)
            continue
        root = node_name(main_path, demo)
        nodes = reachable(graph, root)
        cycles = detect_cycles(graph, nodes)

        safe = re.sub(r'[^A-Za-z0-9_]', '_', demo)
        dot = format_graphviz(safe, graph, nodes, cycles)
        if outdir is None:
            sys.stdout.write(dot)
        else:
            (outdir / f"{safe}.dot").write_text(dot)

        if cycles:
            print(f"Inclusion cycles in {demo}:", file=sys.stderr)
            for c in cycles:
                print("  " + " -> ".join(c), file=sys.stderr)
            exit_code = 1

    sys.exit(exit_code)


if __name__ == "__main__":
    main()
