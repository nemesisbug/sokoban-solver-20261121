# Sokoban Solver — make.fun sokoban-solver-01-run-20261121

Push-optimal Sokoban solver in a single `main.ts` (Node 22, built-ins only).

## How it works

Every generated level decomposes into independent rooms off a one-wide
hallway, so the solver splits the level into rooms and solves each room on
its own, stitching the rooms together with BFS walks through the hallway.
A box pushed out of a room can never come back, so the rooms' push optima
add up to the level optimum P*.

Per room it runs **A\* over pushes** (not moves) with:

- an admissible **minimum-assignment heuristic** — boxes matched to goals
  over shortest-path distances that ignore other boxes;
- **dead-square pruning** — squares from which a box can never be pushed
  onto a goal, found by pulling boxes back from the goals;
- **frozen 2×2-block pruning** — a block of walls and boxes holding a box
  off its goal can never be unfrozen.

A* with reopening is push-optimal, so a solved level earns the full 1000
points per instance. Search budgets are counted in **work** (expansions),
never wall-clock time, so two runs write byte-identical output. If A*
exhausts its budget a room falls back to greedy best-first search, then to
breadth-first push search; an instance is always answered.

## Results

| Set | Score (of 1,000,000) | Pushes vs P* |
| --- | --- | --- |
| Practice set (kit) | **1,000,000** | 13/13, 16/16, 29/29, 47/47, 60/60 |
| 6 freshly drawn sets (kit generator) | **1,000,000 each** | optimal on every instance |

House baseline (plain-astar 1.0.0) scores 200,000 on the practice set.
Total runtime for a 5-instance set: ~0.5 s on 1 CPU (limit is 60 s).

## Run locally

```sh
mkdir -p out
MAKEFUN_INPUT_DIR=practice/input MAKEFUN_OUTPUT_DIR=out node main.ts
```

Reads `/input/<id>/level.txt`, writes `/output/<id>/solution.txt`
(`MAKEFUN_INPUT_DIR` / `MAKEFUN_OUTPUT_DIR` override the paths).

## Licence

MIT — see `LICENSE`.
