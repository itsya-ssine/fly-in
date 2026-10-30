"""
Simulation engine for the Fly-in drone routing system.

Ported from the reference fly-in implementation. The path-finding step
(`pathfinder.PathFinder`) produces a *shared pool* of routes from start
to end -- drones are not pre-assigned to one of them. Instead, every
turn, each drone re-scans that shared pool in order and advances along
the first route that offers an available next hop, claiming a zone/link
capacity slot as it goes. Restricted zones are entered as a two-turn
"half move": a drone first occupies the connecting link for one turn,
then completes the move into the zone on the next.

`SimulationEngine` runs a single pass over one fixed pool of routes.
`Simulator` sits on top of it and tries several pools of increasing size
to find whichever one finishes fastest overall.
"""
from typing import Optional, List, Tuple
from models import Graph, Zone, Connection, DroneState, TurnRecord
from pathfinder import PathFinder


class SimulationEngine:
    """
    Core simulation engine.

    Computes the shared minimal-cost path pool once, then advances drones
    turn by turn: each drone scans the pool for the first route offering a
    free next hop, claims it if both the destination zone and the
    connecting link have spare capacity, and otherwise waits for the next
    turn. Drones are processed in id order each turn, and capacity
    counters are updated immediately as each drone's move is decided, so
    earlier drones in a turn effectively get first pick of contested slots.
    """

    def __init__(self, graph: Graph) -> None:
        """
        Initialize the simulation engine.

        Args:
            graph: The parsed drone network graph.

        Raises:
            ValueError: If graph is invalid or missing required components.
        """
        if not isinstance(graph, Graph):
            raise TypeError(f"Expected Graph instance, got {type(graph)}")
        if graph.start_zone is None:
            raise ValueError("Graph has no start zone defined")
        if graph.end_zone is None:
            raise ValueError("Graph has no end zone defined")
        if graph.nb_drones <= 0:
            raise ValueError(
                f"Graph has invalid number of drones: {graph.nb_drones}"
            )

        self.graph = graph
        self.drones: List[DroneState] = []
        self.paths: List[List[Zone]] = []
        self.turn_records: List[TurnRecord] = []
        self.completion_turn: int = 0
        # Only set (to a concrete int) while running in single-path mode;
        # see `run` and `_next_hop`.
        self._one_path_limit: Optional[int] = None

    def setup(self, paths: Optional[List[List[Zone]]] = None) -> None:
        """
        Place drones at the start zone and set the shared path pool.

        Args:
            paths: The pool of routes drones should draw from. If not
                given, falls back to every route tied for the single
                minimal cost (the original default), so this method
                keeps working for callers that don't pick a pool
                themselves.

        Raises:
            RuntimeError: If path finding fails or start/end zones are invalid.
        """
        if self.graph.start_zone is None:
            raise RuntimeError("Graph has no start zone")
        if self.graph.end_zone is None:
            raise RuntimeError("Graph has no end zone")

        start = self.graph.start_zone
        end = self.graph.end_zone

        # Reset runtime capacity counters in case this graph is re-run.
        for zone in self.graph.zones.values():
            zone.occupancy = 0
            zone.incoming = 0
        for conn in self.graph.connections:
            conn.passing = 0

        if paths is not None:
            if not paths:
                raise ValueError("Paths list cannot be empty")
            self.paths = paths
        else:
            try:
                pathfinder = PathFinder(self.graph)
                self.paths = pathfinder.find_all_minimal_cost_paths(start, end)
            except ValueError as e:
                raise RuntimeError(
                    f"Failed to find minimal cost paths: {e}"
                ) from e

        if not self.paths:
            raise RuntimeError("No paths found for simulation")

        # Validate that all paths start at start zone and end at end zone
        for path in self.paths:
            if path[0] != start:
                raise RuntimeError(
                    f"Path starts at {path[0].name}, expected {start.name}"
                )
            if path[-1] != end:
                raise RuntimeError(
                    f"Path ends at {path[-1].name}, expected {end.name}"
                )

        # Drones start parked at the start zone (no capacity slot is
        # claimed for this initial placement, matching the reference
        # implementation -- only zones entered via a move consume one).
        self.drones = [
            DroneState(drone_id=i + 1, current_zone=start)
            for i in range(self.graph.nb_drones)
        ]

        # Ensure start zone occupancy is 0 (drones don't count until they move)
        start.occupancy = 0

    def run(
        self,
        paths: Optional[List[List[Zone]]] = None,
        try_one_path: bool = False
    ) -> List[TurnRecord]:
        """
        Run the full simulation until all drones arrive.

        Args:
            paths: The pool of routes drones should draw from. See
                `setup` for the default when omitted.
            try_one_path: When True, `paths` is expected to be a single
                route run on its own. In that case, entry into any
                restricted zone on the path is capped by that path's
                max-flow bottleneck (see
                `PathFinder.compute_path_bottleneck`) instead of the
                zone's own `max_drones`. This still lets a new drone
                start queuing at the link the same turn the previous
                occupant settles (no lost turns), while guaranteeing
                the zone's true capacity can never be exceeded -- so a
                drone never has to wait more than the one turn its
                half-move is supposed to take once it has entered a
                connection.

        Returns:
            List of TurnRecords, one per simulation turn.

        Raises:
            RuntimeError: If simulation fails or exceeds maximum turns.
        """
        self.setup(paths)

        if try_one_path:
            if not self.paths:
                raise RuntimeError("No paths available for single-path mode")
            self._one_path_limit = PathFinder(self.graph). \
                compute_path_bottleneck(self.paths[0])
        else:
            self._one_path_limit = None

        max_turns = 10000  # safety limit
        self.turn_records = []
        self.completion_turn = 0

        for turn_num in range(1, max_turns + 1):
            record = TurnRecord(turn_number=turn_num)
            conns: List[Connection] = []
            self._execute_turn(record, conns, try_one_path)

            if record.movements:
                record.capture_capacity(self.graph)
                self.turn_records.append(record)

            self.completion_turn = turn_num

            # Check if all drones have arrived
            if all(d.arrived for d in self.drones):
                break

            # Undo this turn's transient link reservations for normal
            # hops only. Restricted-zone transits keep the reservation
            # `_move` made when they started, until `_complete_transit`
            # releases it exactly once on arrival -- see the note where
            # `conns` is populated in `_execute_turn`.
            for conn in conns:
                conn.leave_station()

        if self.completion_turn >= max_turns:
            raise RuntimeError(
                f"Simulation exceeded maximum turns ({max_turns}) "
                "without completing"
            )

        return self.turn_records

    def _next_hop(
        self,
        current_zone: Zone,
        try_one_path: bool
    ) -> Optional[Zone]:
        """
        Find the next zone a drone should move to this turn.

        Scans the shared minimal-cost path pool in order; for the first
        path that contains the given zone, checks whether that path's
        next zone currently has capacity *and* the connecting link
        currently has capacity. Returns the first such zone found, or
        None if no path offers an available move right now.

        Checking only the destination zone's capacity here isn't enough:
        if that zone is open but the link leading into it is already
        saturated (e.g. another drone claimed its one slot this turn), a
        different path in the pool may still offer a free route through a
        different zone/link pair. Skipping a path solely because its link
        is full -- rather than bailing out to "wait" -- is what lets a
        contested hub push more than one drone through per turn whenever
        the pool actually has the parallel capacity to do so.

        For restricted zones in `try_one_path` mode, capacity is checked
        against the path's precomputed bottleneck (`self._one_path_limit`)
        instead of the zone's `max_drones`. That bottleneck is the true
        sustainable throughput of the whole (non-branching) path, so it's
        always <= every zone's own `max_drones` along the way -- a
        tighter, provably-safe cap for running a single route on its own,
        with no alternate path to fall back to. Every drone this lets
        onto the link is therefore guaranteed a free slot on arrival, so
        `_complete_transit` never needs to make it wait.

        Args:
            current_zone: The zone the drone currently occupies.
            try_one_path: Whether this run is single-path mode.

        Returns:
            The next Zone to move into, or None if the drone should wait.
        """
        for path in self.paths:
            try:
                idx = path.index(current_zone)
            except ValueError:
                continue

            if idx + 1 >= len(path):
                continue  # Path ends here (shouldn't happen if not at end)

            next_zone = path[idx + 1]

            # Check zone capacity
            if try_one_path and next_zone.is_restricted():
                if self._one_path_limit is None:
                    raise RuntimeError(
                        "One-path limit not set for restricted zone check"
                    )
                if next_zone.incoming >= self._one_path_limit:
                    continue
            elif not next_zone.can_enter():
                continue

            # Check link capacity
            conn = self.graph.get_connection(current_zone, next_zone)
            if conn is None:
                continue
            if not conn.can_pass():
                continue

            return next_zone

        return None

    def _execute_turn(
        self, record: TurnRecord, conns: List[Connection], try_one_path: bool
    ) -> None:
        """
        Advance every drone by at most one step for this turn.

        Drones in transit across a restricted-zone link always complete
        their move (the slot was already reserved when they started).
        Drones sitting in a zone look for an available next hop and move
        if both the destination zone and the connecting link have room.

        Args:
            record: TurnRecord to fill with this turn's movements.
            conns: List to collect this turn's normal-hop connections in
                (so their reservations can be released at end of turn).
            try_one_path: Whether this run is single-path mode.
        """
        if self.graph.end_zone is None:
            raise RuntimeError("Graph has no end zone")

        end_zone = self.graph.end_zone

        for drone in self.drones:
            if drone.arrived:
                continue

            if drone.in_transit is not None:
                self._complete_transit(drone, record, end_zone)
                continue

            if drone.current_zone is None:
                raise RuntimeError(
                    f"Drone {drone.drone_name} has no current zone"
                )

            next_zone = self._next_hop(drone.current_zone, try_one_path)
            if next_zone is None:
                continue

            conn = self.graph.get_connection(drone.current_zone, next_zone)
            if conn is None:
                continue
            if not conn.can_pass():
                continue

            # Only normal (single-turn) hops need their link reservation
            # undone at the end of this turn -- see run() and the note
            # on `conns` below. A restricted-zone transit's reservation
            # must persist across turns and is released exactly once,
            # by _complete_transit(), when the drone actually arrives.
            if not next_zone.is_restricted():
                conns.append(conn)

            self._move(drone, next_zone, conn, record, end_zone)

    def _complete_transit(
        self, drone: DroneState, record: TurnRecord, end_zone: Zone
    ) -> None:
        """
        Finish a drone's traversal of a restricted-zone link.

        A drone is only ever let onto the link in the first place if its
        slot at the destination is already guaranteed -- either by the
        normal `can_enter()` check, or, in single-path mode, by the
        path's max-flow bottleneck (see `_next_hop`). So completion here
        is unconditional: it always succeeds on the turn immediately
        after the drone started crossing, and a drone never spends more
        than that one turn occupying the connection.

        Args:
            drone: The drone completing its half-move.
            record: TurnRecord to fill with this turn's movements.
            end_zone: The graph's end zone (to detect arrival).

        Raises:
            RuntimeError: If drone state is invalid for transit completion.
        """
        if drone.in_transit is None:
            raise RuntimeError(
                f"Drone {drone.drone_name} has no in-transit state"
            )

        conn, dest_zone = drone.in_transit

        # Validate destination zone
        if dest_zone.name not in self.graph.zones:
            raise RuntimeError(
                f"Destination zone '{dest_zone.name}' not found in graph"
            )

        # Complete the transit
        dest_zone.incoming -= 1
        if dest_zone.incoming < 0:
            raise RuntimeError(
                f"Negative incoming count for zone '{dest_zone.name}'"
            )

        conn.leave_station()
        dest_zone.enter_station()

        drone.current_zone = dest_zone
        drone.in_transit = None

        if dest_zone == end_zone:
            drone.arrived = True

        record.add_movement(drone.drone_name, dest_zone.name)

    def _move(
        self,
        drone: DroneState,
        next_zone: Zone,
        conn: Connection,
        record: TurnRecord,
        end_zone: Zone,
    ) -> None:
        """
        Move a drone one hop along its chosen route.

        Restricted destination zones are entered as a two-turn half move:
        the drone occupies the link this turn and arrives next turn.
        Every other zone type is entered immediately.

        Args:
            drone: The drone being moved.
            next_zone: The destination zone for this hop.
            conn: The connection being used.
            record: TurnRecord to fill with this turn's movements.
            end_zone: The graph's end zone (to detect arrival).

        Raises:
            RuntimeError: If drone state is invalid for the move.
        """
        if drone.current_zone is None:
            raise RuntimeError(
                f"Drone {drone.drone_name} has no current zone"
            )

        if drone.in_transit is not None:
            raise RuntimeError(
                f"Drone {drone.drone_name} is already in transit"
            )

        if next_zone.is_restricted():
            # Two-turn half move
            next_zone.incoming += 1
            drone.current_zone.leave_station()
            drone.current_zone = None
            drone.in_transit = (conn, next_zone)
            conn.enter_station()
            record.add_movement(
                drone.drone_name, f"{conn.zone_a.name}-{conn.zone_b.name}"
            )
        else:
            # Immediate move
            drone.current_zone.leave_station()
            drone.current_zone = next_zone
            next_zone.enter_station()
            conn.enter_station()

            if next_zone == end_zone:
                drone.arrived = True

            record.add_movement(drone.drone_name, next_zone.name)


class Simulator:
    """
    Orchestrates SimulationEngine runs to find the fastest drone plan.

    Routing every drone down only the single cheapest-per-hop route (or
    routes) can under-use the map: if those routes all squeeze through
    one narrow link or zone, drones queue up single-file even when a
    slightly pricier route would let them spread out and finish sooner
    overall. To avoid that, `solve` tries several candidate pools -- the
    single cheapest route, the two cheapest, the three cheapest, and so
    on -- running a full `SimulationEngine` pass for each, and keeps
    whichever pool actually finishes in the fewest turns.

    The number of pools tried is capped at the map's bottleneck (a
    max-flow estimate of how many drones could ever be usefully in
    motion at once -- see `PathFinder.compute_bottleneck`) and at the
    number of drones in the fleet, since a pool with more routes than
    either offers no further opportunity to parallelize.
    """

    def __init__(self, graph: Graph) -> None:
        """
        Initialize the Simulator.

        Args:
            graph: The parsed drone network graph.

        Raises:
            ValueError: If graph is invalid.
        """
        if not isinstance(graph, Graph):
            raise TypeError(f"Expected Graph instance, got {type(graph)}")
        if graph.start_zone is None:
            raise ValueError("Graph has no start zone")
        if graph.end_zone is None:
            raise ValueError("Graph has no end zone")
        if graph.nb_drones <= 0:
            raise ValueError(
                f"Graph has invalid number of drones: {graph.nb_drones}"
            )

        self.graph = graph
        self.best_completion_turn: Optional[int] = None
        self.best_records: List[TurnRecord] = []
        self.best_drones: List[DroneState] = []

    def solve(self) -> Tuple[List[TurnRecord], List[DroneState]]:
        """
        Run a complete drone routing simulation.

        Returns:
            Tuple of (turn_records, final_drone_states) for the pool
            that finished in the fewest turns.

        Raises:
            RuntimeError: If no valid paths are found or simulation fails.
        """
        if self.graph.start_zone is None:
            raise RuntimeError("Graph has no start zone")
        if self.graph.end_zone is None:
            raise RuntimeError("Graph has no end zone")

        pathfinder = PathFinder(self.graph)

        try:
            ranked_paths = pathfinder.rank_paths_by_cost(
                self.graph.start_zone, self.graph.end_zone
            )
        except ValueError as e:
            raise RuntimeError(f"Failed to rank paths: {e}") from e

        if not ranked_paths:
            raise RuntimeError("No paths found from start to end zone")

        pool_sizes_to_try = max(
            1, min(len(ranked_paths), self.graph.nb_drones)
        )

        # Try the single cheapest path with single-path mode enabled
        self._try_pool(ranked_paths[:1], True)

        # Try larger pools
        for pool_size in range(1, pool_sizes_to_try + 1):
            pool = ranked_paths[:pool_size]
            self._try_pool(pool, False)

        if self.best_completion_turn is None:
            raise RuntimeError("No successful simulation completed")

        return self.best_records, self.best_drones

    def _try_pool(self, pool: List[List[Zone]], try_one_path: bool) -> None:
        """Run one SimulationEngine pass over a candidate path pool.

        Keeps the result if it beats the best result found so far.

        Args:
            pool: The candidate pool of routes to simulate.
            try_one_path: Whether this pool is the single-path run;
                forwarded to `SimulationEngine.run`.
        """
        try:
            engine = SimulationEngine(self.graph)
            records = engine.run(pool, try_one_path)

            if (
                self.best_completion_turn is None
                or engine.completion_turn < self.best_completion_turn
            ):
                self.best_completion_turn = engine.completion_turn
                self.best_records = records
                self.best_drones = engine.drones
        except (RuntimeError, ValueError) as e:
            raise RuntimeError(
                "Error while simulation"
            ) from e
