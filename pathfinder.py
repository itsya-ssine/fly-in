"""
Pathfinding for the Fly-in drone routing simulation.

Ported from the reference fly-in implementation. Rather than computing a
single shortest path, this module enumerates every simple route from the
start zone to the end zone and computes each route's total cost.

Handing the simulation engine more than one route to draw from (instead of
a single shortest path) is what lets a drone fleet spread out and clear
bottleneck maps in few turns. Two selections are exposed on top of that
enumeration:

- `find_all_minimal_cost_paths`: every route tied for the single cheapest
  cost. This is the cheapest possible pool *per drone*, but if the
  network's minimal-cost routes all squeeze through the same narrow
  link/zone, it can under-use capacity that a slightly pricier route
  would unlock.
- `rank_paths_by_cost` + `compute_bottleneck`: every route ranked cheapest
  first, alongside an estimate of how many routes can usefully run in
  parallel at all. `simulation.Simulator` uses these together to try
  pools of increasing size (cheapest path alone, cheapest two, ...) and
  keep whichever pool actually finishes in the fewest turns -- since
  spreading drones over several slightly-more-expensive-per-hop routes
  can finish faster overall than funnelling everyone down the single
  cheapest one.
"""
from typing import List, Tuple, Dict
from models import Graph, Zone, ZoneType


class PathFinder:
    """
    Computes routes and capacity estimates over a drone network graph.

    Instances are bound to a single `Graph`; every method operates on
    that graph and (except `compute_bottleneck`) an explicit start/end
    zone pair, mirroring how `SimulationEngine` is bound to a graph.
    """

    def __init__(self, graph: Graph) -> None:
        """
        Initialize the PathFinder with a graph.

        Args:
            graph: The parsed drone network graph.

        Raises:
            TypeError: If graph is not a Graph instance.
            ValueError: If graph has no zones or connections.
        """
        if not isinstance(graph, Graph):
            raise TypeError(f"Expected Graph instance, got {type(graph)}")
        if not graph.zones:
            raise ValueError("Graph has no zones defined")
        if not graph.connections:
            raise ValueError("Graph has no connections defined")
        if graph.start_zone is None:
            raise ValueError("Graph has no start zone defined")
        if graph.end_zone is None:
            raise ValueError("Graph has no end zone defined")

        self.graph = graph

    def build_cost_graph(self) -> Dict[str, List[Tuple[Zone, int]]]:
        """
        Build an adjacency list keyed by zone name, with per-edge costs.

        Faithful to the reference algorithm, the cost of traversing a
        connection is always taken from the zone type of the
        connection's second endpoint (``zone_b``, i.e. whichever zone
        was written second in the map file) -- the same cost is used
        for both directions of travel. This is a quirk of the original
        implementation, preserved here so the two engines select the
        same minimal-cost routes.

        Connections leading into a blocked zone are omitted entirely: a
        blocked zone's cost() is -1, and leaving that edge in the
        adjacency list would let a DFS "discount" a route by walking
        through a zone no drone can ever actually enter, corrupting
        every cost comparison built on top of this graph.

        Returns:
            Mapping of zone name to a list of (neighbor_zone, cost)
            tuples.
        """
        adjacency: Dict[str, List[Tuple[Zone, int]]] = {}

        for conn in self.graph.connections:
            # Add forward direction if destination is not blocked
            if conn.zone_b.zone_type != ZoneType.BLOCKED:
                cost_b = conn.zone_b.zone_type.cost()
                adjacency.setdefault(
                    conn.zone_a.name, []
                ).append((conn.zone_b, cost_b))

            # Add reverse direction if destination is not blocked
            if conn.zone_a.zone_type != ZoneType.BLOCKED:
                cost_a = conn.zone_a.zone_type.cost()
                adjacency.setdefault(
                    conn.zone_b.name, []
                ).append((conn.zone_a, cost_a))

        return adjacency

    def enumerate_paths_by_cost(
        self,
        start: Zone,
        end: Zone,
    ) -> List[Tuple[int, int, List[Zone]]]:
        """
        Enumerate every simple path from start to end with its cost.

        Uses a DFS-style stack search: every simple path (no zone
        visited twice) is explored and its total cost accumulated edge
        by edge.

        Args:
            start: Starting zone.
            end: Destination zone.

        Returns:
            List of (real_cost, priority_aware_cost, path) tuples, in
            DFS discovery order.
            `real_cost` counts turns treating PRIORITY zones as 1 turn,
            NORMAL as 1, and RESTRICTED as 2. `priority_aware_cost` is
            the original weighted sum using `ZoneType.cost()` (PRIORITY
            zones count as 0 there).

        Raises:
            ValueError: If no path exists from start to end, or if start
                or end zones are invalid.
        """
        if start not in self.graph.zones.values():
            raise ValueError(f"Start zone '{start.name}' not found in graph")
        if end not in self.graph.zones.values():
            raise ValueError(f"End zone '{end.name}' not found in graph")
        if start == end:
            raise ValueError("Start and end zones must be different")

        adjacency = self.build_cost_graph()

        # Validate that start and end are reachable
        if start.name not in adjacency:
            raise ValueError(f"Start zone '{start.name}' has no connections")
        if end.name not in adjacency:
            raise ValueError(f"End zone '{end.name}' has no connections")

        # stack stores (path, real_cost_so_far, weighted_cost_so_far)
        stack: List[Tuple[List[Zone], int, int]] = [([start], 0, 0)]
        found: List[Tuple[int, int, List[Zone]]] = []

        while stack:
            path, real_cost_so_far, weighted_cost_so_far = stack.pop()
            current = path[-1]

            if current == end:
                found.append((real_cost_so_far, weighted_cost_so_far, path))
                continue

            for neighbor, weighted_cost in adjacency.get(current.name, []):
                if neighbor in path:
                    continue  # keep paths simple (no revisits)

                # real cost treats PRIORITY as 1 turn instead of 0
                if neighbor.zone_type == ZoneType.PRIORITY:
                    real_increment = 1
                else:
                    real_increment = neighbor.zone_type.cost()

                stack.append((
                    path + [neighbor],
                    real_cost_so_far + real_increment,
                    weighted_cost_so_far + weighted_cost,
                ))

        if not found:
            raise ValueError(
                f"No path found from '{start.name}' to '{end.name}'. "
                "The graph may be disconnected or have no valid routes."
            )

        return found

    def find_all_minimal_cost_paths(
        self,
        start: Zone,
        end: Zone,
    ) -> List[List[Zone]]:
        """
        Enumerate every simple path and keep the cheapest ones.

        Args:
            start: Starting zone.
            end: Destination zone.

        Returns:
            List of minimal-cost paths, each a list of zones from
            start to end.

        Raises:
            ValueError: If no path exists from start to end.
        """
        found = self.enumerate_paths_by_cost(start, end)

        if not found:
            raise ValueError(
                f"No paths found from '{start.name}' to '{end.name}'"
            )

        # keep legacy behaviour: minimal by the original weighted cost
        best_cost = min(weighted for _, weighted, _ in found)
        return [path for _, weighted, path in found if weighted == best_cost]

    def rank_paths_by_cost(
        self,
        start: Zone,
        end: Zone,
    ) -> List[List[Zone]]:
        """
        Enumerate every simple path from start to end, cheapest first.

        Ties (equal-cost paths) keep their DFS discovery order relative
        to one another, since a stable sort is used.

        Args:
            start: Starting zone.
            end: Destination zone.

        Returns:
            List of paths ordered from cheapest to most expensive.

        Raises:
            ValueError: If no path exists from start to end.
        """
        found = self.enumerate_paths_by_cost(start, end)

        if not found:
            raise ValueError(
                f"No paths found from '{start.name}' to '{end.name}'"
            )

        # sort by real cost (number of turns) first, then by the
        # priority-aware weighted cost so that priority-heavy routes
        # are used as a tiebreaker.
        found.sort(key=lambda item: (item[0], item[1]))
        return [path for _, _, path in found]

    def compute_path_bottleneck(self, path: List[Zone]) -> int:
        """Compute the max flow (bottleneck) achievable along a single path.

        The path is considered in isolation from the rest of the graph.

        A simple path has no branching, so its max flow from start to end
        is just the smallest capacity anywhere along the chain: the
        tightest zone's `max_drones` (start/end zones excluded, since they
        never consume a slot) or the tightest connection's
        `max_link_capacity`. Full Edmonds-Karp isn't needed here -- that's
        only required once the network can branch.

        Used as a hard concurrency cap when running the single cheapest
        path on its own: it tells us exactly how many drones can safely be
        in flight toward a restricted zone at once without ever exceeding
        that zone's real capacity, so a drone is never sent into a
        connection unless its arrival slot is already guaranteed.

        Args:
            path: A single simple path, start zone to end zone.

        Returns:
            The bottleneck capacity of the path, at least 1.

        Raises:
            ValueError: If path is invalid or empty.
        """
        if not path:
            raise ValueError("Path cannot be empty")
        if len(path) < 2:
            raise ValueError(
                f"Path must contain at least 2 zones, got {len(path)}"
            )

        limits: List[int] = []

        # Add zone capacity limits (exclude start and end zones)
        for zone in path:
            if not (zone.is_start or zone.is_end):
                limits.append(zone.max_drones)

        # Add connection capacity limits
        for a, b in zip(path, path[1:]):
            conn = self.graph.get_connection(a, b)
            if conn is None:
                raise ValueError(
                    f"No connection found between '{a.name}' and '{b.name}'"
                )
            limits.append(conn.max_link_capacity)

        if not limits:
            return 1  # All capacities are unbounded or zero

        bottleneck = min(limits)
        return max(1, bottleneck)
