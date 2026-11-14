#!/usr/bin/env python3
"""Generate a maze in the Moving AI .map format used by maps/.

The maze is carved with a randomized depth-first search (recursive
backtracker), which gives long winding corridors and many dead ends. Some
dead ends are then opened into a neighbouring corridor, so that there is
more than one route between two points: BFS and A* can take a shortcut
that DFS does not.

By default corridors and walls are one cell wide, so a maze of n x n cells
is a (2n + 1) x (2n + 1) map. --corridor and --wall widen them: each maze
cell then takes corridor x corridor map cells, with walls of the given
thickness between them.

The two maps in maps/maze/ were made with:
    python3 tools/maze/make_maze.py
    python3 tools/maze/make_maze.py --size 11 --corridor 16 --wall 4 --seed 7 -o maps/maze/maze_wide.map
"""

import argparse
import random
from pathlib import Path

DEFAULT_OUTPUT = Path(__file__).resolve().parents[2] / "maps" / "maze" / "maze.map"
STEPS = [(-1, 0), (1, 0), (0, -1), (0, 1)]


def carve(size, rng):
    """Carve a perfect maze. Returns the grid as a list of lists of booleans
    (True = free)."""
    side = 2 * size + 1
    free = [[False] * side for _ in range(side)]
    visited = [[False] * size for _ in range(size)]

    stack = [(0, 0)]
    visited[0][0] = True
    free[1][1] = True
    while stack:
        r, c = stack[-1]
        options = [
            (r + dr, c + dc)
            for dr, dc in STEPS
            if 0 <= r + dr < size and 0 <= c + dc < size and not visited[r + dr][c + dc]
        ]
        if not options:
            stack.pop()
            continue
        nr, nc = rng.choice(options)
        visited[nr][nc] = True
        free[r + nr + 1][c + nc + 1] = True  # the wall between the two cells
        free[2 * nr + 1][2 * nc + 1] = True
        stack.append((nr, nc))
    return free


def add_loops(free, size, rng, fraction):
    """Open a wall at a random share of the dead ends, joining them to a
    neighbouring corridor."""
    for r in range(size):
        for c in range(size):
            y, x = 2 * r + 1, 2 * c + 1
            open_sides = sum(free[y + dr][x + dc] for dr, dc in STEPS)
            if open_sides != 1 or rng.random() >= fraction:
                continue
            walls = [
                (dr, dc)
                for dr, dc in STEPS
                if not free[y + dr][x + dc] and 0 <= r + dr < size and 0 <= c + dc < size
            ]
            if walls:
                dr, dc = rng.choice(walls)
                free[y + dr][x + dc] = True


def scale(free, corridor, wall):
    """Draw a one-cell maze with wider corridors and walls. Odd rows and
    columns of `free` are corridors, even ones are walls."""
    widths = [corridor if i % 2 else wall for i in range(len(free))]
    starts = [sum(widths[:i]) for i in range(len(widths))]
    side = sum(widths)
    out = [[False] * side for _ in range(side)]
    for i, row in enumerate(free):
        for j, cell in enumerate(row):
            if not cell:
                continue
            for y in range(starts[i], starts[i] + widths[i]):
                for x in range(starts[j], starts[j] + widths[j]):
                    out[y][x] = True
    return out


def write_map(free, path):
    side = len(free)
    lines = ["type octile", f"height {side}", f"width {side}", "map"]
    lines += ["".join("." if cell else "@" for cell in row) for row in free]
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text("\n".join(lines) + "\n")


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--size", type=int, default=64, help="maze cells per side (default 64)")
    parser.add_argument("--seed", type=int, default=2023, help="random seed (default 2023)")
    parser.add_argument("--loops", type=float, default=0.15, help="share of dead ends to open (default 0.15)")
    parser.add_argument("--corridor", type=int, default=1, help="corridor width in map cells (default 1)")
    parser.add_argument("--wall", type=int, default=1, help="wall thickness in map cells (default 1)")
    parser.add_argument("-o", "--output", type=Path, default=DEFAULT_OUTPUT)
    args = parser.parse_args()

    rng = random.Random(args.seed)
    free = carve(args.size, rng)
    add_loops(free, args.size, rng, args.loops)
    free = scale(free, args.corridor, args.wall)
    write_map(free, args.output)
    print(f"Wrote {args.output} ({len(free)} x {len(free)})")


if __name__ == "__main__":
    main()
