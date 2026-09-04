*This project has been created as part of the 42 curriculum by brouane.*

# Fly-in — Drone Fleet Routing Simulation

## Description

**Fly-in** routes a fleet of autonomous drones from a shared `start_hub` to a shared
`end_hub` across a graph of zones (`hub`s connected by `connection`s), in the fewest
possible simulation turns, while respecting per-zone occupancy limits and per-zone
movement costs (`normal`, `priority`, `restricted`, `blocked`).

The whole thing is written in pure, object-oriented Python — **no `networkx`,
`graphlib`, or any other graph library**, as required by the subject. Path lengths,
turn scheduling and zone/turn occupancy are computed by hand with a Dijkstra-style
search built on top of Python's built-in [`heapq`](https://docs.python.org/3/library/heapq.html)
module.

The project is split into small, single-responsibility classes:

```
fly_in_takeoff.py     # CLI entry point / orchestration
parsing.py             # Parser -> reads and validates the map file
zone_reachability.py   # ZoneReachability -> pre-flight sanity check
algo_start.py           # Algo -> pathfinding + turn-by-turn scheduling
drones.py               # Drone / drone_creator -> drone bookkeeping
simulation_printer.py   # SimulationPrinter -> D<id>-<zone> turn log
display.py              # Display -> Tkinter graphical playback
custom_types.py         # Shared type aliases (ParsedData, HubDict, ...)
maps/                   # Easy / medium / hard / challenger test maps
```

## Instructions

Requirements: **Python 3.10+**, [`uv`](https://docs.astral.sh/uv/), and a Tk-enabled
Python install (Tkinter ships with most CPython builds). A `drone.png` icon file must
be present in the working directory — it's loaded as the window icon by `Display`.

```bash
make install       # uv sync — installs dev dependencies (flake8, mypy)
make run           # uv run python3 fly_in_takeoff.py  (defaults to maps/easy/mine.txt)
make debug         # same, but through pdb
make lint          # flake8 . && mypy . (project's required flag set)
make lint-strict   # flake8 . && mypy . --strict
make clean         # remove __pycache__ / .mypy_cache
```

To run a specific map:

```bash
uv run python3 fly_in_takeoff.py --map maps/hard/02_capacity_hell.txt
```

The program first prints the turn-by-turn move log to the terminal, then opens the
Tkinter window so you can replay the same simulation visually.

## How the pathfinding works

### Two Dijkstra passes, for two different jobs

We rely on `heapq` (a binary min-heap) twice, for two different purposes:

1. **`Algo.shortest_path_search` — feasibility check.**
   A classic Dijkstra over the static graph: starting from `start_hub`, it repeatedly
   pulls the cheapest *unvisited* zone from a heap, relaxes its neighbours, and stops
   once `end_hub` is popped. Edge weight = the *destination* zone's movement cost
   (`normal`/`priority` = 1, `restricted` = 2, `blocked` = unreachable). This runs once,
   before any drone is scheduled, purely to fail fast with `"No solution to the map"`
   if `end_hub` is provably unreachable.

2. **`Algo.find_drone_path` — the real per-drone router.**
   This is the core of the project: a **time-expanded Dijkstra**. Instead of searching
   over zones alone, it searches over `(zone, turn)` states, so two drones trying to use
   the same zone on the same turn are naturally treated as different, competing paths.
   Each heap entry is `(turn, -priority_count, zone, path_so_far)`:
   - `turn` is the primary sort key, so the search always expands the earliest-arriving
     state first — this is what makes it a *shortest-path-in-time* search rather than a
     plain graph search.
   - `-priority_count` (negated) is the tie-breaker: among paths that arrive at the same
     turn, the min-heap pops the one that has passed through more `priority` zones
     first, matching the subject's "priority zones should be preferred" rule without
     needing a second full pass.
   - From each popped state the search explores two kinds of transitions: **wait one
     turn** in place (if the current zone still has capacity next turn), and **move to
     each neighbour** (if that neighbour isn't `blocked` and has capacity). A `(zone,
     turn)` state is only ever expanded once (`visited` set), which keeps the search
     terminating even on graphs with cycles (see *Edge cases* below).
   - The very first popped state whose zone is `end_hub` is returned — this is the
     drone's full path, one zone name per turn.

Each drone runs its own `find_drone_path` search *after* the previous drones' paths
have already been written into the shared turn-by-turn occupancy snapshots, so later
drones automatically route around zones/turns that earlier drones have already claimed
— this is how "distribute drones across multiple paths" and "strategic waiting" fall
out of a single, reused search routine instead of needing a separate conflict-resolution
pass.

### Zone types, concretely

| Type         | Cost to enter | Behaviour in the search |
|--------------|:---:|---|
| `normal`     | 1 turn | Default. Treated as a normal edge relaxation. |
| `priority`   | 1 turn | Same cost as `normal`, but increments the tie-break counter described above, so equally-fast routes through `priority` zones are preferred. |
| `restricted` | 2 turns | The neighbour transition costs `stay = 2` instead of `1`: the search jumps straight from turn `T` to turn `T+2` for that move — there is no intermediate "waiting on the connection" state, matching the rule that a drone **must** land in the destination on the very next scheduling step and can't linger mid-flight. Capacity is checked for the destination zone at *both* `T+1` and `T+2` so the zone stays reserved for the whole transit, not just on arrival. |
| `blocked`    | — | Excluded outright (`continue`) whenever it shows up as a neighbour — never enters the heap, so no drone can ever route through it. |

### Occupancy, capacity and simultaneous movement

`Algo` keeps a list of per-turn snapshots, `zones_at_turnes`: a `{zone_name: [drone_ids]}`
mapping for every turn. `start_hub` and `end_hub` are exempt from capacity checks (as
required — all drones may start together, any number may be delivered), every other
zone is checked against its `max_drones` (default 1). `commit_path` walks a drone's
finished path and, turn by turn, removes the drone from its old zone and adds it to the
new one in that turn's snapshot (and propagates the change forward until the drone's
position changes again), so several drones can move on the same turn as long as each
individual zone's capacity is respected turn-by-turn.

### Output format

`SimulationPrinter` walks consecutive snapshot pairs and, for every drone that changed
zone, emits `D<id>-<zone>`. Drones sitting in a `restricted` zone's transit are only
printed once they've actually landed (their move is deferred a turn via
`pending_restricted_turn`), and a drone stops being tracked the turn it reaches
`end_hub`. Turns with no movement (or once every drone is delivered) simply aren't
printed, matching the "drones that do not move are omitted" rule.

## Example input and expected output

The two examples below were produced by actually running the program against the
maps shipped in `maps/`, not written by hand — you can reproduce either one with
`uv run python3 fly_in_takeoff.py --map <path>`.

### 1. `maps/easy/01_linear_path.txt` — a single-file corridor

Input file:

```
# Easy Level 1: Simple linear path
nb_drones: 2

start_hub: start 0 0 [color=green]
hub: waypoint1 1 0 [color=blue]
hub: waypoint2 2 0 [color=blue]
end_hub: goal 3 0 [color=red]

connection: start-waypoint1
connection: waypoint1-waypoint2
connection: waypoint2-goal
```

Every intermediate zone defaults to `max_drones=1`, so the two drones can't walk the
corridor side by side — the terminal log this produces is:

```
D1-waypoint1
D1-waypoint2 D2-waypoint1
D1-goal D2-waypoint2
D2-goal
Total turns: 4
```

`D1` claims `waypoint1` on turn 1; `D2` can only follow it in on turn 2, once `D1` has
already moved on to `waypoint2` — exactly the "strategic waiting" and single-zone
capacity behaviour described above, without either drone ever colliding.

### 2. `maps/easy/mine.txt` — a fork with `restricted` zones

Input file (trimmed to the relevant lines):

```
nb_drones: 3

start_hub: start 0 0 [color=green]
hub: waypoint1 1 -1 [color=red max_drones=1]
hub: waypoint2 2 0 [zone=restricted color=red max_drones=1]
hub: waypoint3 1 1  [zone=restricted color=red max_drones=1]
hub: waypoint4 3 -1  [zone=restricted color=red max_drones=1]
hub: waypoint5 2 1  [color=red max_drones=1]
end_hub: goal 3 0 [color=red]

connection: start-waypoint1
connection: start-waypoint3
connection: waypoint1-waypoint2
connection: waypoint2-waypoint4
connection: waypoint4-goal
connection: waypoint3-waypoint5
connection: waypoint5-goal
```

Terminal log:

```
D1-waypoint3 D3-waypoint1
D1-waypoint3 D3-waypoint2
D1-waypoint5 D2-waypoint3 D3-waypoint2
D1-goal D2-waypoint3 D3-waypoint4
D2-waypoint5 D3-waypoint4
D2-goal D3-goal
Total turns: 6
```

Notice `D1-waypoint3` printed on two consecutive turns: `waypoint3` is a `restricted`
zone, so entering it costs 2 turns — the drone is "in transit" on the first line and
only counted as arrived on the second, matching the movement-cost rule from the *Zone
types* table above. Drones also fan out across both branches of the fork (`waypoint1`
vs `waypoint3`) instead of queueing on a single path, which is the tie-break/priority
logic and the shared-occupancy bookkeeping working together.



`Display` opens a maximized Tkinter window and draws the whole graph once:

- Every hub is placed on the canvas using its `x`/`y` grid coordinates (scaled to the
  window size) and drawn as a colored circle (`fill` = the hub's `color` metadata,
  validated through `winfo_rgb` so a bad color name fails loudly instead of silently).
- Every `connection` is drawn as a dashed line between the two hub circles' centers,
  sent to the back (`tag_lower`) so it never covers the hub markers or their labels.
- Each hub has a text label (its name) above it and a live text field below it showing
  the IDs of the drones currently occupying it, wrapped every 10 characters so it stays
  readable even with many drones stacked in one zone.
- **Left / Right arrow keys** (and matching **Prev / Next** buttons in the bottom-right
  corner) step the `current_turn` index backward/forward through the pre-computed
  `zones_at_turnes` snapshots; only the drone-ID text objects are updated on each step
  (`_update_zone_texts`), so scrubbing through turns is cheap — the graph itself is
  built once. A turn counter in the top-right corner tracks where you are in the replay.

This turns the simulation from a wall of `D1-roof1 D2-corridorA ...` text into a
scrubbable, spatial replay of the whole fleet, which is what makes it easy to actually
see conflicts, waiting, and capacity bottlenecks happening zone by zone.

## Edge cases handled

- **Unreachable map** — `ZoneReachability` does a plain DFS/BFS from `start_hub` before
  any pathfinding runs and raises `ValueError` listing every zone that can't be reached
  at all (regardless of `blocked`/capacity), so those problems are reported clearly
  rather than surfacing as a confusing pathfinding failure later.
- **No route to `end_hub`** — even if the map is otherwise connected, `blocked` zones
  can still cut off `end_hub`; the upfront Dijkstra feasibility check catches this and
  raises `"No solution to the map"` before any drone is created.
- **Invalid `nb_drones`** — zero or negative values print a warning and fall back to 1
  drone instead of crashing.
- **Invalid `max_drones` / `max_link_capacity`** — non-positive values print a warning
  and fall back to the default of 1.
- **`max_drones` on `start_hub` / `end_hub`** — silently ignored (forced to unlimited)
  per the subject, even if the map author sets it explicitly.
- **Duplicate hub names or coordinates** — rejected with a clear `TypeError`.
- **Duplicate connections** — `a-b` and `b-a` are recognized as the same edge (edges are
  sorted before comparison) and rejected.
- **Invalid zone type / hub name with `-` or space / malformed field order** — all
  rejected with descriptive `TypeError`s naming the offending rule.
- **Declared-but-unused zones** — printed as a non-fatal warning rather than an error,
  since an isolated hub doesn't break the simulation as long as everything required is
  still reachable.
- **All top-level failures** (`FileNotFoundError`, `PermissionError`, `ValueError`,
  `TypeError`, `OSError`) are caught in `main()` and reported as a single
  `<<ERROR DETECTED>>: ...` line instead of a raw traceback.
- **Cycles in the graph** — the `(zone, turn)` `visited` set in `find_drone_path`
  guarantees every state is only expanded once, so circular connections (see
  `maps/medium/02_circular_loop.txt`) can't cause an infinite search.

### Known limitation

`max_link_capacity` is fully parsed and validated, but the current scheduler only
enforces **zone** occupancy (`max_drones`) turn-by-turn — connection-level throughput
limits are not yet taken into account when routing drones. This is a natural next
optimization: adding an edge-usage counter per `(connection, turn)` alongside the
existing zone-usage counters.

## Complexity

Both searches are plain Dijkstra variants driven by `heapq`, so each individual search
is roughly `O((V + E) log V)`. `find_drone_path` runs once per drone, and its state
space is `(zone × turn)` instead of just `zone`, so its practical cost scales with the
number of drones and the length of the simulation horizon rather than the raw graph
size alone — later drones search a bit more because earlier drones' reservations
shrink the set of free `(zone, turn)` states they can use.

## Resources

- Python [`heapq`](https://docs.python.org/3/library/heapq.html) documentation.
- Dijkstra's algorithm — general reference: [Wikipedia](https://en.wikipedia.org/wiki/Dijkstra%27s_algorithm).
- [Tkinter](https://docs.python.org/3/library/tkinter.html) official documentation for
  `Canvas`, `Button`, and event binding.
- [`uv`](https://docs.astral.sh/uv/) documentation for dependency management.
- **AI usage**: An AI assistant was used to get a plain-language refresher on how
  `heapq` works (`heappush`/`heappop`/`heapify`/`nsmallest`) and on the general shape of
  Dijkstra's algorithm (maintaining a cost/came-from table and always expanding the
  globally cheapest unvisited node) before implementing `Algo` by hand — no algorithm
  or graph code was generated or copied directly from the assistant. It was also used
  to help proofread this README for clarity.
