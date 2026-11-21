// Sokoban solver for make.fun sokoban-solver-01-run-20261121.
//
// Approach (written for this bounty): every level decomposes into independent
// rooms off a 1-wide hallway, so each room is solved on its own and the walks
// between rooms are stitched with plain BFS. Per room we run A* over PUSHES
// (not moves) with an admissible minimum-assignment heuristic (boxes matched
// to goals over shortest-path distances that ignore other boxes), dead-square
// pruning (a box that can never be pushed onto a goal) and frozen 2x2-block
// pruning. A* with reopening is push-optimal, so a solved level scores the
// full 1000 points per instance. Budgets are counted in work (expansions),
// never in wall-clock time, so two runs write identical bytes.
//
// Fallbacks, in order, per room: A*, greedy best-first on the same
// heuristic, breadth-first push search. If every search gives up, the
// instance is still answered (empty solution) rather than risking the
// whole run's 60 s wall budget. No clock, no randomness, no environment
// reads anywhere in the search.
import { mkdirSync, readdirSync, readFileSync, writeFileSync } from 'node:fs';
import { join } from 'node:path';

const IN_DIR = process.env.MAKEFUN_INPUT_DIR ?? '/input';
const OUT_DIR = process.env.MAKEFUN_OUTPUT_DIR ?? '/output';

// Fixed expansion budgets (work, never time). The generator guarantees each
// room is solvable by plain push-BFS within 250k states; A* with the
// assignment heuristic expands far fewer, so these caps are generous.
const ASTAR_BUDGET = 120_000;
const GREEDY_BUDGET = 120_000;
const BFS_BUDGET = 200_000;
// Global cap across the whole run, so a pathological set can never blow the
// 60 s wall budget: once spent, remaining rooms fall straight to greedy.
const GLOBAL_BUDGET = 1_500_000;

let spent = 0;

const DIRS: [number, number, string, string][] = [
  [0, -1, 'u', 'U'],
  [0, 1, 'd', 'D'],
  [-1, 0, 'l', 'L'],
  [1, 0, 'r', 'R'],
];

interface Level {
  w: number;
  h: number;
  wall: Uint8Array;
  goals: number[];
  boxes: number[];
  player: number;
}

function parseLevel(text: string): Level {
  const rows = text.split('\n').filter((r) => r.length > 0);
  const h = rows.length;
  const w = Math.max(...rows.map((r) => r.length));
  const wall = new Uint8Array(w * h).fill(1);
  const goals: number[] = [];
  const boxes: number[] = [];
  let player = -1;
  rows.forEach((row, y) => {
    for (let x = 0; x < row.length; x++) {
      const ch = row[x];
      const i = y * w + x;
      if (ch === '#') continue;
      wall[i] = 0;
      if (ch === '.' || ch === '*' || ch === '+') goals.push(i);
      if (ch === '$' || ch === '*') boxes.push(i);
      if (ch === '@' || ch === '+') player = i;
    }
  });
  for (let y = 0; y < h; y++) {
    for (let x = 0; x < w; x++) {
      if (x === 0 || y === 0 || x === w - 1 || y === h - 1) wall[y * w + x] = 1;
    }
  }
  return { w, h, wall, goals, boxes: boxes.sort((a, b) => a - b), player };
}

// Squares from which a box can be pushed onto some goal, found by pulling
// boxes back from the goals over open floor. A box anywhere else is dead.
function liveSquares(lv: Level): Uint8Array {
  const { w, wall, goals } = lv;
  const live = new Uint8Array(wall.length);
  const queue = [...goals];
  for (const g of goals) live[g] = 1;
  for (let qi = 0; qi < queue.length; qi++) {
    const x = queue[qi];
    for (const [dx, dy] of DIRS) {
      const d = dy * w + dx;
      const y = x - d;
      if (!wall[y] && !wall[y - d] && !live[y]) {
        live[y] = 1;
        queue.push(y);
      }
    }
  }
  return live;
}

// Flood-fill the player's reachable region from `from` with `occ` blocking;
// marks it in `seen` with `stamp` and returns its lowest square.
function flood(lv: Level, occ: Uint8Array, from: number, seen: Int32Array, stamp: number, stack: Int32Array): number {
  const { w, wall } = lv;
  let low = from;
  let n = 0;
  stack[n++] = from;
  seen[from] = stamp;
  while (n > 0) {
    const p = stack[--n];
    if (p < low) low = p;
    for (let k = 0; k < 4; k++) {
      const q = k === 0 ? p - w : k === 1 ? p + w : k === 2 ? p - 1 : p + 1;
      if (wall[q] === 0 && occ[q] === 0 && seen[q] !== stamp) {
        seen[q] = stamp;
        stack[n++] = q;
      }
    }
  }
  return low;
}

// True when the box just pushed to `t` sits in a frozen 2x2 block of walls
// and boxes holding a box off its goal.
function frozenBlock(lv: Level, occ: Uint8Array, goal: Uint8Array, t: number): boolean {
  const { w, wall } = lv;
  for (const a of [t, t - 1, t - w, t - w - 1]) {
    const b = a + 1;
    const c = a + w;
    const d = c + 1;
    if (!(wall[a] || occ[a]) || !(wall[b] || occ[b]) || !(wall[c] || occ[c]) || !(wall[d] || occ[d])) continue;
    if ((occ[a] && !goal[a]) || (occ[b] && !goal[b]) || (occ[c] && !goal[c]) || (occ[d] && !goal[d])) return true;
  }
  return false;
}

// Shortest walk from `from` to `to` around walls and boxes (BFS, fixed
// direction order, so deterministic). Lower-case letters.
function walkTo(lv: Level, occ: Uint8Array, from: number, to: number): string {
  const { w, wall } = lv;
  if (from === to) return '';
  const prev = new Int32Array(wall.length).fill(-1);
  const how = new Array<string>(wall.length);
  const queue = [from];
  prev[from] = from;
  for (let qi = 0; qi < queue.length; qi++) {
    const p = queue[qi];
    if (p === to) break;
    for (const [dx, dy, letter] of DIRS) {
      const q = p + dy * w + dx;
      if (wall[q] || occ[q] || prev[q] !== -1) continue;
      prev[q] = p;
      how[q] = letter;
      queue.push(q);
    }
  }
  if (prev[to] === -1) return '';
  let s = '';
  for (let p = to; p !== from; p = prev[p]) s = how[p] + s;
  return s;
}

// All permutations of 0..n-1 (n <= 4 here), in lexicographic order.
function permutations(n: number): number[][] {
  const out: number[][] = [];
  const a = [...Array(n).keys()];
  const gen = (k: number): void => {
    if (k === 1) {
      out.push([...a]);
      return;
    }
    for (let i = 0; i < k; i++) {
      gen(k - 1);
      const j = k % 2 === 1 ? 0 : i;
      const t = a[j];
      a[j] = a[k - 1];
      a[k - 1] = t;
    }
  };
  gen(n);
  return out;
}

interface SearchResult {
  pushes: number;
  moves: string;
}

// Push search over one (sub-)level. mode 0 = A* (optimal), 1 = greedy
// best-first on h, 2 = breadth-first over pushes (optimal). Returns null
// when the expansion budget runs out or no solution exists.
function pushSearch(lv: Level, mode: number, budget: number): SearchResult | null {
  const { w, wall, goals } = lv;
  const size = wall.length;
  const n = lv.boxes.length;
  const goal = new Uint8Array(size);
  for (const g of goals) goal[g] = 1;
  const live = liveSquares(lv);
  if (lv.boxes.some((b) => !live[b])) return null;
  const solved = (boxes: number[]): boolean => boxes.every((b) => goal[b]);

  // BFS distances from each goal over open floor (boxes ignored): a lower
  // bound on the pushes any box needs to reach that goal.
  const distFrom: Int32Array[] = goals.map((g) => {
    const dist = new Int32Array(size).fill(-1);
    dist[g] = 0;
    const q = [g];
    for (let qi = 0; qi < q.length; qi++) {
      const p = q[qi];
      for (const [dx, dy] of DIRS) {
        const s = p + dy * w + dx;
        if (!wall[s] && dist[s] === -1) {
          dist[s] = dist[p] + 1;
          q.push(s);
        }
      }
    }
    return dist;
  });
  const perms = permutations(n);
  // Minimum-cost assignment of boxes to goals; Infinity when no box can
  // reach its goal (dead state). Admissible: each push moves one box one
  // square closer to some goal by at most one step along its shortest path.
  const heuristic = (boxes: number[]): number => {
    let best = Infinity;
    for (const perm of perms) {
      let sum = 0;
      for (let i = 0; i < n; i++) {
        const d = distFrom[perm[i]][boxes[i]];
        if (d < 0) {
          sum = Infinity;
          break;
        }
        sum += d;
      }
      if (sum < best) best = sum;
    }
    return best;
  };

  const pack = (boxes: number[]): number => {
    let p = 0;
    for (let i = n - 1; i >= 0; i--) p = p * 256 + boxes[i];
    return p;
  };
  const unpack = (p: number): number[] => {
    const boxes: number[] = [];
    for (let i = 0; i < n; i++) {
      boxes.push(p % 256);
      p = Math.floor(p / 256);
    }
    return boxes;
  };

  const occ = new Uint8Array(size);
  const seen = new Int32Array(size);
  const mark = new Int32Array(size);
  const stack = new Int32Array(size);
  let stamp = 0;

  // States: packed boxes, player square after the push, parent, push from/to.
  const stPack: number[] = [pack(lv.boxes)];
  const stPlayer: number[] = [lv.player];
  const stParent: number[] = [-1];
  const stFrom: number[] = [-1];
  const stTo: number[] = [-1];
  const stG: number[] = [0];
  const stKey: number[] = [0];
  const bestG = new Map<number, number>();
  for (const b of lv.boxes) occ[b] = 1;
  const startKey = pack(lv.boxes) * 256 + flood(lv, occ, lv.player, seen, ++stamp, stack);
  for (const b of lv.boxes) occ[b] = 0;
  stKey[0] = startKey;
  bestG.set(startKey, 0);

  // Heap of [f, -g, counter, state]: smallest f first, deeper g first on a
  // tie, then insertion order (fully deterministic).
  const heap: [number, number, number, number][] = [];
  let counter = 0;
  const less = (a: [number, number, number, number], b: [number, number, number, number]): boolean =>
    a[0] < b[0] || (a[0] === b[0] && (a[1] < b[1] || (a[1] === b[1] && a[2] < b[2])));
  const heapPush = (e: [number, number, number, number]): void => {
    let i = heap.length;
    heap.push(e);
    while (i > 0) {
      const p = (i - 1) >> 1;
      if (!less(heap[i], heap[p])) break;
      const t = heap[i];
      heap[i] = heap[p];
      heap[p] = t;
      i = p;
    }
  };
  const heapPop = (): [number, number, number, number] => {
    const top = heap[0];
    const last = heap.pop()!;
    if (heap.length > 0) {
      heap[0] = last;
      for (let i = 0; ; ) {
        let c = 2 * i + 1;
        if (c >= heap.length) break;
        if (c + 1 < heap.length && less(heap[c + 1], heap[c])) c++;
        if (!less(heap[c], heap[i])) break;
        const t = heap[i];
        heap[i] = heap[c];
        heap[c] = t;
        i = c;
      }
    }
    return top;
  };

  const h0 = heuristic(lv.boxes);
  if (h0 === Infinity) return null;
  heapPush([mode === 2 ? 0 : h0, 0, counter++, 0]);
  let expanded = 0;
  let found = solved(lv.boxes) ? 0 : -1;

  while (heap.length > 0 && found < 0) {
    if (expanded >= budget || spent >= GLOBAL_BUDGET) return null;
    const [, negG, , s] = heapPop();
    const g = -negG;
    // Stale entry: a cheaper route to this state's key was found since.
    if ((bestG.get(stKey[s]) ?? Infinity) < g) continue;
    const boxes = unpack(stPack[s]);
    expanded++;
    spent++;
    for (const b of boxes) occ[b] = 1;
    const here = ++stamp;
    flood(lv, occ, stPlayer[s], seen, here, stack);
    for (let bi = 0; bi < n && found < 0; bi++) {
      const b = boxes[bi];
      for (const [dx, dy] of DIRS) {
        const d = dy * w + dx;
        const t = b + d;
        // Player must stand at b - d, destination must be open and live.
        if (seen[b - d] !== here || wall[t] || occ[t] || !live[t]) continue;
        occ[b] = 0;
        occ[t] = 1;
        const frozen = frozenBlock(lv, occ, goal, t);
        let key = -1;
        let next: number[] | null = null;
        let h = Infinity;
        if (!frozen) {
          next = boxes.slice();
          next[bi] = t;
          next.sort((x, y) => x - y);
          h = heuristic(next);
          if (h !== Infinity) key = pack(next) * 256 + flood(lv, occ, b, mark, ++stamp, stack);
        }
        occ[t] = 0;
        occ[b] = 1;
        if (frozen || h === Infinity || key < 0) continue;
        const ng = g + 1;
        const prev = bestG.get(key);
        if (prev !== undefined && prev <= ng) continue;
        bestG.set(key, ng);
        const idx = stPack.length;
        stPack.push(pack(next!));
        stPlayer.push(b);
        stParent.push(s);
        stFrom.push(b);
        stTo.push(t);
        stG.push(ng);
        stKey.push(key);
        if (solved(next!)) {
          found = idx;
          break;
        }
        const f = mode === 0 ? ng + h : mode === 1 ? h : ng;
        // BFS (mode 2) breaks ties by insertion order (FIFO); A*/greedy
        // prefer deeper states on a tie.
        heapPush([f, mode === 2 ? 0 : -ng, counter++, idx]);
      }
    }
    for (const b of boxes) occ[b] = 0;
  }
  if (found < 0) return null;

  // Rebuild the move string: walk to each push square, then push.
  const path: number[] = [];
  for (let s = found; s > 0; s = stParent[s]) path.push(s);
  path.reverse();
  let moves = '';
  let player = lv.player;
  let boxes = lv.boxes;
  for (const s of path) {
    const nextBoxes = unpack(stPack[s]);
    const from = boxes.find((b) => !nextBoxes.includes(b))!;
    const to = nextBoxes.find((b) => !boxes.includes(b))!;
    const d = to - from;
    const dir = DIRS.find(([dx, dy]) => dy * w + dx === d)!;
    for (const b of boxes) occ[b] = 1;
    moves += walkTo(lv, occ, player, from - d);
    for (const b of boxes) occ[b] = 0;
    moves += dir[3];
    player = from;
    boxes = nextBoxes;
  }
  return { pushes: path.length, moves };
}

interface Room {
  squares: number[];
  boxes: number[];
  goals: number[];
}

// Split a level into rooms: cut the hallway row's dead squares (a box there
// reaches no goal) out of the floor; each remaining part holding boxes is a
// room. Null when the level does not decompose cleanly.
function roomsOf(lv: Level): Room[] | null {
  const { w, wall, goals, boxes, player } = lv;
  const live = liveSquares(lv);
  const row = Math.floor(player / w);
  const cut = new Uint8Array(wall.length);
  for (let i = row * w; i < (row + 1) * w; i++) if (!wall[i] && !live[i]) cut[i] = 1;
  if (boxes.some((b) => cut[b])) return null;
  const part = new Int32Array(wall.length).fill(-1);
  const rooms: Room[] = [];
  for (let s = 0; s < wall.length; s++) {
    if (wall[s] || cut[s] || part[s] >= 0) continue;
    const squares = [s];
    part[s] = rooms.length;
    for (let i = 0; i < squares.length; i++) {
      for (const [dx, dy] of DIRS) {
        const q = squares[i] + dy * w + dx;
        if (!wall[q] && !cut[q] && part[q] < 0) {
          part[q] = rooms.length;
          squares.push(q);
        }
      }
    }
    rooms.push({ squares, boxes: [], goals: [] });
  }
  for (const b of boxes) rooms[part[b]].boxes.push(b);
  for (const g of goals) if (part[g] >= 0) rooms[part[g]].goals.push(g);
  if (goals.some((g) => part[g] < 0) || rooms.some((r) => r.boxes.length !== r.goals.length)) return null;
  return rooms.filter((r) => r.boxes.length > 0);
}

function endOf(w: number, from: number, moves: string): number {
  let p = from;
  for (const ch of moves.toLowerCase()) p += ch === 'u' ? -w : ch === 'd' ? w : ch === 'l' ? -1 : 1;
  return p;
}

// Solve one room (as a sub-level with the other rooms' boxes as walls),
// trying A*, then greedy, then push-BFS.
function solveRoom(lv: Level, room: Room, others: Room[], player: number): SearchResult | null {
  const wall = lv.wall.slice();
  for (const other of others) {
    for (const b of other.boxes) wall[b] = 1;
  }
  const sub: Level = {
    w: lv.w,
    h: lv.h,
    wall,
    goals: room.goals.slice().sort((a, b) => a - b),
    boxes: room.boxes.slice().sort((a, b) => a - b),
    player,
  };
  return (
    pushSearch(sub, 0, ASTAR_BUDGET) ??
    pushSearch(sub, 1, GREEDY_BUDGET) ??
    pushSearch(sub, 2, BFS_BUDGET)
  );
}

function solveLevel(text: string): string {
  const lv = parseLevel(text);
  const rooms = roomsOf(lv);
  if (rooms !== null) {
    let player = lv.player;
    let moves = '';
    const solvedRooms: Room[] = [];
    for (const room of rooms) {
      // Rooms already solved keep their boxes on their goals (as walls);
      // rooms not yet attempted keep their boxes where they stand.
      const others: Room[] = rooms
        .filter((r) => r !== room)
        .map((r) => (solvedRooms.includes(r) ? { ...r, boxes: [...r.goals] } : r));
      const res = solveRoom(lv, room, others, player);
      if (res === null) return '';
      moves += res.moves;
      player = endOf(lv.w, player, res.moves);
      solvedRooms.push(room);
    }
    return moves;
  }
  // No clean decomposition: search the whole level the same way.
  const res =
    pushSearch(lv, 0, ASTAR_BUDGET) ?? pushSearch(lv, 1, GREEDY_BUDGET) ?? pushSearch(lv, 2, BFS_BUDGET);
  return res === null ? '' : res.moves;
}

for (const id of readdirSync(IN_DIR).sort()) {
  const moves = solveLevel(readFileSync(join(IN_DIR, id, 'level.txt'), 'utf8'));
  mkdirSync(join(OUT_DIR, id), { recursive: true });
  writeFileSync(join(OUT_DIR, id, 'solution.txt'), `${moves}\n`);
}
