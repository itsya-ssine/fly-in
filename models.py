"""Models for the Fly-in drone routing simulation.

Defines Zone, Connection, Drone, and Graph data structures.
"""
from __future__ import annotations
from enum import Enum
from typing import Optional, List, Tuple, Dict
from pydantic import BaseModel, Field, model_validator


class ZoneType(str, Enum):
    """Zone movement type with associated turn cost."""

    NORMAL = "normal"
    BLOCKED = "blocked"
    RESTRICTED = "restricted"
    PRIORITY = "priority"

    def cost(self) -> int:
        """Return movement cost in turns for entering a zone of this type.

        Mirrors the reference fly-in cost table: normal zones cost 1 turn,
        priority zones are free (0), restricted zones cost 2, and blocked
        zones are non-traversable (-1).
        """
        return {
            ZoneType.NORMAL: 1,
            ZoneType.PRIORITY: 0,
            ZoneType.RESTRICTED: 2,
            ZoneType.BLOCKED: -1,
        }[self]


class Zone(BaseModel):
    """Represents a zone (node) in the drone network graph."""

    name: str
    x: int
    y: int
    zone_type: ZoneType = Field(default=ZoneType.NORMAL)
    color: Optional[str] = Field(default=None)
    max_drones: int = Field(default=1, ge=1)
    is_start: bool = Field(default=False)
    is_end: bool = Field(default=False)
    occupancy: int = Field(default=0)
    incoming: int = Field(default=0)

    model_config = {
        "frozen": False,  # Allow mutations
        "extra": "forbid",  # Don't allow extra fields
    }

    def __hash__(self) -> int:
        """Hash based on name."""
        return hash(self.name)

    def __eq__(self, other: object) -> bool:
        """Equality based on name."""
        if not isinstance(other, Zone):
            return False
        return self.name == other.name

    def __repr__(self) -> str:
        """Return the string representation."""
        return f"Zone({self.name})"

    @model_validator(mode="after")
    def validate_start_end(self) -> Zone:
        """Validate that start and end flags are mutually exclusive."""
        if self.is_start and self.is_end:
            raise ValueError("Zone cannot be both start and end")
        return self

    def is_restricted(self) -> bool:
        """Return True if this zone is a restricted zone."""
        return self.zone_type == ZoneType.RESTRICTED

    def can_enter(self) -> bool:
        """Return True if a drone may enter this zone right now.

        The end zone always accepts drones. Otherwise both the settled
        occupancy and the in-transit ("incoming") count must stay below
        the zone's capacity.
        """
        if self.is_end:
            return True
        return self.occupancy + self.incoming < self.max_drones

    def enter_station(self) -> None:
        """Record a drone settling into this zone."""
        self.occupancy += 1

    def leave_station(self) -> None:
        """Record a drone leaving this zone."""
        self.occupancy -= 1


class Connection(BaseModel):
    """Represents a bidirectional edge between two zones."""

    zone_a: Zone
    zone_b: Zone
    max_link_capacity: int = Field(default=1, ge=1)
    passing: int = Field(default=0)

    model_config = {
        "frozen": False,
        "extra": "forbid",
    }

    def other(self, zone: Zone) -> Zone:
        """Return the zone on the other end of this connection."""
        if zone == self.zone_a:
            return self.zone_b
        return self.zone_a

    def involves(self, zone: Zone) -> bool:
        """Return True if this connection touches the given zone."""
        return zone == self.zone_a or zone == self.zone_b

    def can_pass(self) -> bool:
        """Return True if a drone can start traversing this link right now."""
        return self.passing < self.max_link_capacity

    def enter_station(self) -> None:
        """Record a drone starting to traverse this link."""
        self.passing += 1

    def leave_station(self) -> None:
        """Record a drone finishing its traversal of this link."""
        self.passing -= 1

    def __hash__(self) -> int:
        """Hash based on sorted zone names for bidirectionality."""
        names = tuple(sorted([self.zone_a.name, self.zone_b.name]))
        return hash(names)

    def __eq__(self, other: object) -> bool:
        """Equality ignoring direction."""
        if not isinstance(other, Connection):
            return False
        return (
            (self.zone_a == other.zone_a and self.zone_b == other.zone_b)
            or (self.zone_a == other.zone_b and self.zone_b == other.zone_a)
        )

    def __repr__(self) -> str:
        """Return the string representation."""
        return f"Connection({self.zone_a.name}-{self.zone_b.name})"

    @model_validator(mode="after")
    def validate_zones(self) -> Connection:
        """Validate that zones are different."""
        if self.zone_a == self.zone_b:
            raise ValueError("Connection cannot connect a zone to itself")
        return self


class DroneState(BaseModel):
    """Represents the state of a drone at a given simulation turn.

    Drones no longer carry a fixed pre-assigned route: each turn the
    simulation re-scans the shared pool of minimal-cost paths and advances
    the drone along whichever one currently has capacity.
    """

    drone_id: int = Field(..., ge=1)
    current_zone: Optional[Zone] = Field(default=None)
    in_transit: Optional[Tuple[Connection, Zone]] = Field(default=None)
    arrived: bool = Field(default=False)

    model_config = {
        "frozen": False,
        "extra": "forbid",
    }

    @property
    def drone_name(self) -> str:
        """Return formatted drone name like D1."""
        return f"D{self.drone_id}"

    def is_in_transit(self) -> bool:
        """Return True if drone is currently traversing a connection."""
        return self.in_transit is not None


class TurnRecord(BaseModel):
    """Records drone movements in a single simulation turn."""

    turn_number: int
    movements: List[Tuple[str, str]] = Field(default_factory=list)
    # (drone_name, destination_label)
    zone_capacity: Dict[str, Tuple[int, int]] = Field(default_factory=dict)
    # zone_name -> (used, max_drones)
    connection_capacity: Dict[str, Tuple[int, int]] = Field(
        default_factory=dict
    )
    # "zoneA-zoneB" -> (used, max_link_capacity)

    def capture_capacity(self, graph: "Graph") -> None:
        """Snapshot zone and connection capacity usage for this turn."""
        for name, zone in graph.zones.items():
            used = zone.occupancy + zone.incoming
            self.zone_capacity[name] = (used, zone.max_drones)
        for conn in graph.connections:
            key = f"{conn.zone_a.name}-{conn.zone_b.name}"
            self.connection_capacity[key] = (
                conn.passing, conn.max_link_capacity
            )

    def add_movement(self, drone_name: str, dest_label: str) -> None:
        """Record a drone movement."""
        self.movements.append((drone_name, dest_label))

    def format_output(self) -> str:
        """Format this turn's movements in the required output format."""
        if not self.movements:
            return ""
        parts = [f"{dn}-{dl}" for dn, dl in sorted(self.movements)]
        return " ".join(parts)


class Graph(BaseModel):
    """Represents the full drone network graph."""

    zones: Dict[str, Zone] = Field(default_factory=dict)
    connections: List[Connection] = Field(default_factory=list)
    start_zone: Optional[Zone] = Field(default=None)
    end_zone: Optional[Zone] = Field(default=None)
    nb_drones: int = Field(default=0, ge=0)

    model_config = {
        "frozen": False,
        "extra": "forbid",
    }

    def get_zone(self, name: str) -> Optional[Zone]:
        """Retrieve a zone by name."""
        return self.zones.get(name)

    def get_neighbors(self, zone: Zone) -> List[Tuple[Zone, Connection]]:
        """Return all zones reachable from the given zone with connections."""
        neighbors: List[Tuple[Zone, Connection]] = []
        for conn in self.connections:
            if conn.involves(zone):
                other = conn.other(zone)
                if other.zone_type != ZoneType.BLOCKED:
                    neighbors.append((other, conn))
        return neighbors

    def get_connection(
            self,
            zone_a: Zone,
            zone_b: Zone
    ) -> Optional[Connection]:
        """Return the connection between two zones if it exists."""
        for conn in self.connections:
            if conn.involves(zone_a) and conn.involves(zone_b):
                return conn
        return None

    @model_validator(mode="after")
    def validate_start_end_exist(self) -> Graph:
        """Validate that start and end zones exist in the zones dict."""
        if self.start_zone and self.start_zone.name not in self.zones:
            raise ValueError(
                f"Start zone '{self.start_zone.name}' not found in zones")
        if self.end_zone and self.end_zone.name not in self.zones:
            raise ValueError(
                f"End zone '{self.end_zone.name}' not found in zones")
        return self
