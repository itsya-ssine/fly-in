*This project has been created as part of the 42 curriculum by yelmajdo.*

# fly-in

## Description

**fly-in** is a drone-routing simulator: given a map of "zones" (hubs) connected by links, and a fleet of drones sitting at a single start zone, it computes how those drones should move, turn by turn, to all reach a single end zone in as few turns as possible — without ever exceeding the capacity of a zone or a link.

- **Zones** (nodes) can be `normal`, `priority` (free to enter), `restricted` (costs extra and takes two turns to enter), or `blocked` (impassable), and each has a maximum number of drones it can hold at once.
- **Connections** (edges) between zones are bidirectional and have their own maximum "in-flight" capacity.
- The **simulator** decides, turn by turn, which drone moves where, respecting all zone/link capacity constraints, until every drone has reached the end zone.
- A **terminal visualizer** renders the whole run with colors: a zone legend, per-turn movements, and a final summary.

The goal is to find a routing strategy that finishes in the minimum number of turns, then present that result in a way that's pleasant and easy to read in a terminal.

## Instructions

### Requirements

- Python 3.10+
- `pydantic`
- `matplotlib`
- Optional, for development: `flake8` and `mypy`

### Installation

```bash
make install
```

This installs the runtime dependencies, `pydantic` and `matplotlib`.

For linting tools (optional):

```bash
make install-lint
```

### Running the simulation

```bash
python3 main.py <map_file>
```

or, using the default sample map bundled in the `Makefile`:

```bash
make run
```

- `<map_file>` — path to a text file describing the drone network (see [Map file format](#map-file-format) below). It's the only argument the CLI accepts.

Example:

```bash
python3 main.py maps/medium/02_circular_loop.txt
```

### Debugging

```bash
make debug
```

Runs the program under `pdb` on the default map.

### Linting

```bash
make lint          # flake8 + mypy (relaxed)
make lint-strict    # flake8 + mypy --strict
```

### Cleaning

```bash
make clean
```

Removes `__pycache__`, `.mypy_cache`, and compiled `.pyc` files.

### Map file format

A map file is a plain text file, parsed line by line (blank lines and lines starting with `#` are ignored):

```
nb_drones: <positive integer>

start_hub: <name> <x> <y> [color=<name>]
end_hub:   <name> <x> <y> [color=<name>]
hub:       <name> <x> <y> [zone=<normal|blocked|restricted|priority> color=<name> max_drones=<n>]

connection: <zone1>-<zone2> [max_link_capacity=<n>]
```

Rules enforced by the parser:

- `nb_drones` must appear before any hub definition.
- Exactly one `start_hub` and exactly one `end_hub` are required.
- Zone names cannot contain spaces or dashes (dashes are reserved as the connection separator).
- Metadata in `[...]` is optional; `zone` defaults to `normal`, `max_drones` defaults to `1`, `max_link_capacity` defaults to `1`.
- Duplicate zone names and duplicate connections (in either direction) are rejected.

Example map:

```
nb_drones: 4

start_hub: A 0 0
end_hub:   F 6 0

hub: B 2 0 [zone=normal max_drones=1]
hub: C 4 0 [zone=restricted]
hub: D 2 2 [zone=priority]
hub: E 4 2 [zone=blocked]

connection: A-B
connection: B-C
connection: A-D
connection: D-C [max_link_capacity=2]
connection: C-F
```

## Example input and expected output

Given the map above, running:

```bash
python3 main.py maps/example.txt
```

produces a colored terminal report, for example:

```
============================================================
  ✈  FLY-IN
============================================================
  Drones:         4
  Zones:          6
  Connections:    5
  Start:          A (0,0)
  End:            F (6,0)
============================================================

  ZONES
    ● A [START]
    ● F [END]
    ● B
    ● C [RESTRICTED]
    ● D [PRIORITY]
    ● E [BLOCKED]

  SIMULATION

  Turn   1  D1-D
  Turn   2  D1-D-C D2-D
  Turn   3  D1-C D2-D-C D3-D
  Turn   4  D1-F D2-C D3-D-C D4-D
  Turn   5  D2-F D3-C D4-D-C
  Turn   6  D3-F D4-C
  Turn   7  D4-F

============================================================
  ✓ All drones delivered!
  Total turns:            7
  Total drone movements:  16
  Avg drones/turn:        2.3
============================================================
```

Each turn line lists every drone that moved that turn as `<drone>-<destination>`. A destination like `D-C` means the drone is mid-transit into a `restricted` zone (occupying the link on its way from `D` to `C`, a two-turn half-move); a plain destination like `D` or `C` means it landed there this turn.

## Algorithm and implementation strategy

The pipeline is split into three independent stages — parsing, path finding, and simulation — glued together by `main.py`.

### 1. Parsing (`parser.py`, `models.py`)

The map file is read into a `Graph` (a `pydantic` model holding `Zone` and `Connection` objects). Validation happens both syntactically (line format, metadata tokens) and semantically (zone/connection references must exist, start/end must be unique, zone names must be unambiguous). All the domain rules — a zone's movement cost, its remaining capacity, a connection's remaining capacity — live as methods directly on the `Zone`/`Connection` models, keeping `Simulator` and `PathFinder` free of ad-hoc bookkeeping.

### 2. Path finding (`pathfinder.py`)

Rather than computing a single shortest path, `PathFinder` enumerates **every simple path** from start to end via an explicit DFS stack (no recursion), accumulating cost edge by edge. Costs follow the reference cost table (`normal`=1, `priority`=0, `restricted`=2, `blocked`=impassable), applied to the zone being entered.

Two views are built on top of that enumeration:

- **`find_all_minimal_cost_paths`** — every path tied for the single cheapest cost.
- **`rank_paths_by_cost`** — every path ordered cheapest-first, used to build candidate *pools* of routes.

A **max-flow computation (Edmonds–Karp)** over a zone-splitting flow network (`compute_bottleneck`) estimates the largest number of drones that could plausibly be in motion at once, given zone and link capacities. This isn't a perfect model of the turn-based simulation (it ignores the two-turn restricted-zone half-move, for instance), but it's a cheap, dependency-free bound used purely to decide how many candidate pools are worth trying.

### 3. Simulation (`simulation.py`)

`SimulationEngine` runs a single pass over one fixed pool of routes: every turn, each drone (processed in ID order) scans the pool for the first route offering a free next hop — checking *both* the destination zone's capacity and the connecting link's capacity — and claims it if available, otherwise waits. Entering a `restricted` zone is modeled as a two-turn "half move": the drone occupies the link for one turn, then completes the move on the next.

`Simulator` sits on top of this: since routing everyone down only the single cheapest path can bottleneck the network even when a slightly pricier alternate route would let drones spread out, it tries increasingly large pools — the cheapest path alone, the two cheapest, the three cheapest, and so on, capped by the bottleneck estimate and by the fleet size — runs a full `SimulationEngine` pass for each pool, and keeps whichever pool finishes in the fewest turns.

This "try several pool sizes and keep the best" strategy is the core design decision of the project: it trades extra computation (multiple full simulation passes) for solution quality, without needing a more complex joint path-selection-and-scheduling algorithm.

### 4. CLI (`main.py`)

`FlyInCLI` wires the three stages together: it validates the argument count and file existence, then runs `Parser → Simulator → TerminalVisualizer` in sequence, catching each stage's failure modes independently (parse errors, simulation errors, visualization errors) and reporting them with a distinct, actionable message and a non-zero exit code.

## Visual representation features

`visualizer.py` implements a `TerminalVisualizer` that turns the raw simulation trace into a readable ANSI-colored terminal report:

- **Header** — drone count, zone count, connection count, and the start/end zones with coordinates, colored to match their zone type.
- **Zone legend** — every zone listed with a colored bullet matching its type (`normal`, `restricted`, `priority`, `blocked`) or an explicit custom color from the map file, plus `[START]`/`[END]` tags and capacity annotations (`cap=n`) where relevant. A zone colored `rainbow` gets a per-character gradient instead of one flat color.
- **Per-turn movements** — each turn is printed on its own line, with every drone's move shown as `<colored drone id>-<destination>`; drone colors are assigned by ID so the same drone stays visually identifiable across the whole run.
- **Summary** — whether all drones arrived (and which ones didn't, if not), total turns to completion, total drone movements, and average drones moved per turn.

Coloring zones and drones consistently makes it possible to visually trace a single drone's route and to immediately spot bottlenecks (e.g. many drones queuing at a `restricted` zone) without reading through raw coordinates or logs.

## Test maps

The `maps/` directory bundles a graded set of challenge maps (`easy`, `medium`, `hard`, `challenger`), each targeting a specific pathfinding or scheduling difficulty — dead ends, circular loops, capacity bottlenecks, and combinations thereof. See `maps/README.md` for the full breakdown of each map, the challenge types they cover, and expected turn-count benchmarks.

## Resources

- [Pydantic documentation](https://docs.pydantic.dev/) — data validation and modeling
- [Depth-first search (DFS)](https://www.geeksforgeeks.org/dsa/depth-first-search-or-dfs-for-a-graph/) — used to enumerate all simple paths between start and end
- Python [`typing`](https://docs.python.org/3/library/typing.html) and `mypy` documentation — for the strict static typing used throughout the codebase

### AI usage

AI assistance was used during this project for:

- Drafting and refining docstrings and inline comments to explain non-obvious design decisions.
- Reviewing edge cases in the parser and in the simulation engine.
- Generating this README.