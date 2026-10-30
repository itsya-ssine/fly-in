"""Parser for the Fly-in drone map file format.

Reads .txt map files and constructs a Graph object.
"""
from typing import Dict, Set, Optional, Tuple
from models import Graph, Zone, ZoneType, Connection


class ParseError(Exception):
    """Raised when the map file has a syntax or semantic error."""

    def __init__(self, line_num: int, message: str) -> None:
        """Initialize with the offending line number and message."""
        super().__init__(f"Line {line_num}: {message}")
        self.line_num = line_num


class Parser():
    """
    Parser for Fly-in drone map files.

    Parses .txt map files and constructs a Graph object representing
    the drone network with zones, connections, and drone counts.
    """

    # Metadata keys accepted on zone lines (hub / start_hub / end_hub)
    ZONE_METADATA_KEYS = {'zone', 'color', 'max_drones'}
    # Metadata keys accepted on connection lines
    CONNECTION_METADATA_KEYS = {'max_link_capacity'}

    def _parse_metadata(self, raw: str, line_num: int) -> Dict[str, str]:
        """
        Parse the inside of a metadata block, e.g. 'zone=normal color=red'.

        Args:
            raw: The bracket content (without the surrounding brackets).
            line_num: Line number for error reporting.

        Returns:
            Dictionary of key-value metadata pairs.

        Raises:
            ParseError: If metadata syntax is invalid.
        """
        result: Dict[str, str] = {}
        if not raw.strip():
            return result

        tokens = raw.strip().split()
        for token in tokens:
            if token.count('=') != 1:
                raise ParseError(
                    line_num, f"Invalid metadata token: '{token}'. "
                    "Expected format: key=value"
                )
            key, _, value = token.partition('=')
            key = key.strip()
            value = value.strip()
            if not key or not value:
                raise ParseError(
                    line_num, f"Malformed metadata: '{token}'. "
                    "Both key and value must be non-empty"
                )
            if key in result:
                raise ParseError(
                    line_num, f"Duplicate metadata key: '{key}'"
                )
            result[key] = value
        return result

    def _extract_metadata_block(
        self, tail: str, line_num: int
    ) -> Optional[str]:
        """Validate and extract the trailing metadata block of a line.

        `tail` (whatever follows the mandatory positional fields of a
        line) must be either empty or a single well-formed '[...]'
        block; its inner content is returned.

        Args:
            tail: The remaining text after the positional fields.
            line_num: Line number for error reporting.

        Returns:
            The metadata content (string between the brackets), or None
            if there was no metadata block at all.

        Raises:
            ParseError: If metadata block format is invalid.
        """
        tail = tail.strip()
        if not tail:
            return None

        if not (tail.startswith('[') and tail.endswith(']')):
            raise ParseError(
                line_num,
                "Invalid metadata: expected '[...]' block"
            )

        inner = tail[1:-1]
        # A stray '[' or ']' inside means either a nested block or more
        # than one metadata block was supplied - both are invalid.
        if '[' in inner or ']' in inner:
            raise ParseError(
                line_num,
                "Invalid metadata: nested or multiple metadata blocks "
                "are not allowed"
            )

        return inner

    def _parse_zone_line(
        self,
        prefix: str,
        rest: str,
        line_num: int,
        is_start: bool,
        is_end: bool
    ) -> Zone:
        """
        Parse a hub, start_hub, or end_hub line into a Zone.

        Expected format: <name> <x> <y> [metadata]
        Metadata is optional for every zone type.

        Args:
            prefix: The line prefix (hub/start_hub/end_hub).
            rest: Everything after the colon.
            line_num: Line number for error reporting.
            is_start: Whether this is a start zone.
            is_end: Whether this is an end zone.

        Returns:
            A Zone instance.

        Raises:
            ParseError: If zone syntax or metadata is invalid.
        """
        # maxsplit=3 keeps a metadata block containing spaces intact as
        # the 4th chunk, instead of splitting it apart.
        parts = rest.strip().split(None, 3)
        if len(parts) < 3:
            raise ParseError(
                line_num,
                f"Expected: {prefix}: <name> <x> <y> [metadata]. "
                f"Got: '{rest}'"
            )

        name, x_str, y_str = parts[0], parts[1], parts[2]
        tail = parts[3] if len(parts) == 4 else ""

        if not name:
            raise ParseError(line_num, "Zone name cannot be empty")
        if '-' in name or ' ' in name:
            raise ParseError(
                line_num,
                f"Zone name '{name}' must not contain dashes or spaces"
            )

        try:
            x = int(x_str)
            y = int(y_str)
        except ValueError as e:
            raise ParseError(
                line_num,
                f"Zone coordinates must be integers: {e}"
            ) from e

        meta_content = self._extract_metadata_block(tail, line_num)
        metadata = (
            self._parse_metadata(meta_content, line_num)
            if meta_content is not None else {}
        )

        invalid_keys = set(metadata) - self.ZONE_METADATA_KEYS
        if invalid_keys:
            raise ParseError(
                line_num,
                f"Invalid metadata key(s) for zone: "
                f"{', '.join(sorted(invalid_keys))}. "
                f"Allowed keys: {', '.join(sorted(self.ZONE_METADATA_KEYS))}"
            )

        zone_type_str = metadata.get('zone', 'normal')
        try:
            zone_type = ZoneType(zone_type_str)
        except ValueError as e:
            raise ParseError(
                line_num,
                f"Invalid zone type '{zone_type_str}'. "
                f"Must be one of: normal, blocked, restricted, priority"
            ) from e

        # start_hub and end_hub have no capacity limit: any max_drones
        # metadata on them is ignored entirely (not validated, and its
        # presence/value is never an error). Zone.max_drones is a
        # required positive int downstream (and both the simulation's
        # can_enter() and the pathfinder's bottleneck estimate already
        # special-case is_start/is_end to bypass this field), so we
        # simply leave it at its harmless default here rather than
        # trying to parse whatever the user wrote.
        if is_start or is_end:
            max_drones: int = int(1e+19)
        else:
            max_drones = 1
            if 'max_drones' in metadata:
                try:
                    max_drones = int(metadata['max_drones'])
                    if max_drones <= 0:
                        raise ValueError(
                            "max_drones must be a positive integer"
                        )
                except ValueError as e:
                    raise ParseError(
                        line_num,
                        f"max_drones must be a positive integer: {e}"
                    ) from e

        color = metadata.get('color', None)
        if color is not None and not color.strip():
            raise ParseError(
                line_num,
                "Color metadata cannot be empty"
            )

        return Zone(
            name=name,
            x=x,
            y=y,
            zone_type=zone_type,
            color=color,
            max_drones=max_drones,
            is_start=is_start,
            is_end=is_end
        )

    def _parse_connection_line(
            self,
            rest: str,
            line_num: int,
            graph: Graph) -> Connection:
        """
        Parse a connection line into a Connection object.

        Expected format: <zone1>-<zone2> [metadata]

        Args:
            rest: Everything after 'connection: '.
            line_num: Line number for error reporting.
            graph: The current graph (to look up zones).

        Returns:
            A Connection instance.

        Raises:
            ParseError: If connection syntax or metadata is invalid.
        """
        # maxsplit=1: the first whitespace-delimited token is the
        # 'zone1-zone2' pair (zone names can't contain spaces), and
        # everything after it is the optional metadata block.
        parts = rest.strip().split(None, 1)
        if not parts:
            raise ParseError(
                line_num,
                "Connection must be 'zone1-zone2'"
            )

        conn_part = parts[0]
        tail = parts[1] if len(parts) == 2 else ""

        meta_content = self._extract_metadata_block(tail, line_num)
        metadata = (
            self._parse_metadata(meta_content, line_num)
            if meta_content is not None else {}
        )

        invalid_keys = set(metadata) - self.CONNECTION_METADATA_KEYS
        if invalid_keys:
            raise ParseError(
                line_num,
                f"Invalid metadata key(s) for connection: "
                f"{', '.join(sorted(invalid_keys))}. "
                "Allowed keys: "
                f"{', '.join(sorted(self.CONNECTION_METADATA_KEYS))}"
            )

        # Use a single dash as separator — zone names cannot contain dashes
        name_parts = conn_part.split('-')
        if len(name_parts) != 2:
            raise ParseError(
                line_num,
                f"Connection must be 'zone1-zone2', got: '{conn_part}'. "
                "Zone names cannot contain dashes"
            )

        name_a, name_b = name_parts[0].strip(), name_parts[1].strip()
        if not name_a or not name_b:
            raise ParseError(
                line_num,
                f"Connection must be 'zone1-zone2', got: '{conn_part}'. "
                "Both zone names must be non-empty"
            )

        zone_a = graph.get_zone(name_a)
        zone_b = graph.get_zone(name_b)

        if zone_a is None:
            raise ParseError(
                line_num,
                f"Unknown zone '{name_a}'. "
                "Zone must be defined before being used in a connection"
            )
        if zone_b is None:
            raise ParseError(
                line_num,
                f"Unknown zone '{name_b}'. "
                "Zone must be defined before being used in a connection"
            )

        if zone_a == zone_b:
            raise ParseError(
                line_num,
                f"Cannot create connection from zone '{name_a}' to itself"
            )

        max_link_capacity = 1
        if 'max_link_capacity' in metadata:
            try:
                max_link_capacity = int(metadata['max_link_capacity'])
                if max_link_capacity <= 0:
                    raise ValueError(
                        "max_link_capacity must be a positive integer"
                    )
            except ValueError as e:
                raise ParseError(
                    line_num,
                    f"max_link_capacity must be a positive integer: {e}"
                ) from e

        return Connection(
            zone_a=zone_a,
            zone_b=zone_b,
            max_link_capacity=max_link_capacity
        )

    def _register_zone(
        self,
        graph: Graph,
        zone: Zone,
        line_num: int,
        seen_coords: Dict[Tuple[int, int], str]
    ) -> None:
        """Add a freshly-parsed zone to the graph.

        Enforces that both its name and its coordinates are unique
        across the whole map.

        Args:
            graph: The graph to add the zone to.
            zone: The zone to register.
            line_num: Line number for error reporting.
            seen_coords: Dictionary of seen coordinates to zone names.

        Raises:
            ParseError: If zone name or coordinates are duplicated.
        """
        if zone.name in graph.zones:
            raise ParseError(
                line_num,
                f"Duplicate zone name '{zone.name}'. "
                "Zone names must be unique"
            )

        coord = (zone.x, zone.y)
        if coord in seen_coords:
            raise ParseError(
                line_num,
                f"Zone coordinates {coord} for '{zone.name}' are already "
                f"used by zone '{seen_coords[coord]}'. "
                "Zone coordinates must be unique"
            )
        seen_coords[coord] = zone.name
        graph.zones[zone.name] = zone

    def parse_map_file(self, filepath: str) -> Graph:
        """
        Parse a drone map file and return a fully constructed Graph.

        Args:
            filepath: Path to the map file.

        Returns:
            A Graph object representing the drone network.

        Raises:
            ParseError: If any syntax or semantic error is found.
            FileNotFoundError: If the file does not exist.
            PermissionError: If the file cannot be read.
            ValueError: If the file is empty or invalid.
        """
        graph = Graph()
        seen_connections: Set[frozenset[str]] = set()
        seen_coords: Dict[Tuple[int, int], str] = {}
        start_count = 0
        end_count = 0
        nb_drones_set = False
        first_content_line = True

        try:
            with open(filepath, 'r', encoding='utf-8') as f:
                lines = f.readlines()
        except FileNotFoundError as e:
            raise FileNotFoundError(
                f"Map file not found: '{filepath}'"
            ) from e
        except PermissionError as e:
            raise PermissionError(
                f"Cannot read map file: '{filepath}'"
            ) from e
        except UnicodeDecodeError as e:
            raise ValueError(
                f"Map file must be UTF-8 encoded: '{filepath}'"
            ) from e

        if not lines:
            raise ValueError(f"Map file is empty: '{filepath}'")

        for line_num, raw_line in enumerate(lines, start=1):
            # Strip inline/trailing comments (comments may start anywhere
            # on the line, not just at column 0), then surrounding
            # whitespace.
            line = raw_line.split('#', 1)[0].strip()

            # Skip empty lines and comment-only lines
            if not line:
                continue

            if first_content_line:
                if not line.startswith('nb_drones:'):
                    raise ParseError(
                        line_num,
                        "The first line of the map must define nb_drones"
                    )
                first_content_line = False

            if line.startswith('nb_drones:'):
                if nb_drones_set:
                    raise ParseError(
                        line_num,
                        "Duplicate nb_drones definition. "
                        "nb_drones can only be defined once"
                    )
                rest = line[len('nb_drones:'):].strip()
                if not rest:
                    raise ParseError(
                        line_num,
                        "nb_drones requires a positive integer value"
                    )
                try:
                    nb = int(rest)
                    if nb <= 0:
                        raise ValueError(
                            "nb_drones must be a positive integer"
                        )
                except ValueError as e:
                    raise ParseError(
                        line_num,
                        f"nb_drones must be a positive integer: {e}"
                    ) from e
                graph.nb_drones = nb
                nb_drones_set = True

            elif line.startswith('start_hub:'):
                rest = line[len('start_hub:'):].strip()
                if not rest:
                    raise ParseError(
                        line_num,
                        "start_hub requires: <name> <x> <y> [metadata]"
                    )
                zone = self._parse_zone_line(
                    'start_hub', rest, line_num, True, False
                )
                self._register_zone(graph, zone, line_num, seen_coords)
                graph.start_zone = zone
                start_count += 1

            elif line.startswith('end_hub:'):
                rest = line[len('end_hub:'):].strip()
                if not rest:
                    raise ParseError(
                        line_num,
                        "end_hub requires: <name> <x> <y> [metadata]"
                    )
                zone = self._parse_zone_line(
                    'end_hub', rest, line_num, False, True
                )
                self._register_zone(graph, zone, line_num, seen_coords)
                graph.end_zone = zone
                end_count += 1

            elif line.startswith('hub:'):
                rest = line[len('hub:'):].strip()
                if not rest:
                    raise ParseError(
                        line_num,
                        "hub requires: <name> <x> <y> [metadata]"
                    )
                zone = self._parse_zone_line(
                    'hub', rest, line_num, False, False
                )
                self._register_zone(graph, zone, line_num, seen_coords)

            elif line.startswith('connection:'):
                if not nb_drones_set:
                    raise ParseError(
                        line_num,
                        "nb_drones must be defined before connections"
                    )
                rest = line[len('connection:'):].strip()
                if not rest:
                    raise ParseError(
                        line_num,
                        "connection requires: <zone1>-<zone2> [metadata]"
                    )
                conn = self._parse_connection_line(rest, line_num, graph)

                # Check for duplicate connections
                key = frozenset([conn.zone_a.name, conn.zone_b.name])
                if key in seen_connections:
                    raise ParseError(
                        line_num,
                        f"Duplicate connection "
                        f"'{conn.zone_a.name}-{conn.zone_b.name}'. "
                        "Connections must be unique"
                    )
                seen_connections.add(key)
                graph.connections.append(conn)

            else:
                raise ParseError(
                    line_num,
                    f"Unrecognized line format: '{line}'. "
                    "Expected: nb_drones, hub, start_hub, end_hub, "
                    "or connection"
                )

        # Validate final state
        if not nb_drones_set:
            raise ParseError(
                0,
                "Missing nb_drones definition. "
                "File must start with 'nb_drones: <number>'"
            )
        if start_count != 1:
            raise ParseError(
                0,
                f"Expected exactly 1 start_hub, found {start_count}. "
                "Exactly one start_hub is required"
            )
        if end_count != 1:
            raise ParseError(
                0,
                f"Expected exactly 1 end_hub, found {end_count}. "
                "Exactly one end_hub is required"
            )

        # Validate graph consistency
        if not graph.connections:
            raise ParseError(
                0,
                "No connections defined. "
                "At least one connection is required for simulation"
            )

        return graph
