"""
Terminal visualizer for the Fly-in drone simulation.
Provides colored output showing drone movements and zone states.
"""
import colorsys
from typing import List, Dict, Optional
import matplotlib.colors as mcolors
from models import Graph, Zone, ZoneType, DroneState
from models import TurnRecord


class TerminalVisualizer:
    """Terminal visualizer for the Fly-in drone simulation."""

    # ANSI color codes
    RESET: str = "\033[0m"
    BOLD: str = "\033[1m"
    DIM: str = "\033[2m"

    # Fallback colors used only when a zone has no explicit `color`
    # metadata (its zone type / start / end status is used instead).
    ZONE_COLORS: Dict[str, str] = {
        "normal": "\033[36m",       # Cyan
        "restricted": "\033[33m",   # Yellow
        "priority": "\033[32m",     # Green
        "blocked": "\033[31m",      # Red
        "start": "\033[92m",        # Bright green
        "end": "\033[93m",          # Bright yellow
    }

    DRONE_COLORS: List[str] = [
        "\033[94m",  # Blue
        "\033[95m",  # Magenta
        "\033[96m",  # Cyan
        "\033[91m",  # Red
        "\033[92m",  # Green
        "\033[93m",  # Yellow
        "\033[97m",  # White
        "\033[35m",  # Dark magenta
    ]

    def __init__(self, graph: Graph) -> None:
        """
        Initialize the visualizer with a graph.

        Args:
            graph: The drone network graph.
        """
        self.graph: Graph = graph

    def _color_to_ansi(self, color: str) -> str:
        """
        Convert any matplotlib-recognized color (named color, hex code,
        'tab:*', 'xkcd:*', grayscale string, etc.) into a 24-bit ANSI
        truecolor foreground escape sequence.

        Args:
            color: A color string as accepted by matplotlib.

        Returns:
            ANSI escape sequence string.

        Raises:
            ValueError: If matplotlib doesn't recognize the color.
        """
        r, g, b = mcolors.to_rgb(color)
        return f"\033[38;2;{round(r * 255)};{round(g * 255)};{round(b * 255)}m"

    def _get_zone_color(self, zone: Zone) -> str:
        """
        Get ANSI color for a zone based on its `color` metadata, falling
        back to its type/start/end status when no color was given.

        Note: for `color=rainbow`, `mcolors.to_rgb` raises ValueError,
        so this falls through to the type/start/end fallback below.
        That's fine here -- this single flat color is only used for
        things like the legend bullet; the zone *name* itself is
        rendered separately via `_colorize_text`, which handles the
        rainbow case per-character.

        Args:
            zone: The zone to get color for.

        Returns:
            ANSI color code string.
        """
        if zone.color:
            try:
                return self._color_to_ansi(zone.color)
            except ValueError:
                pass
        if zone.is_start:
            return self.ZONE_COLORS["start"]
        if zone.is_end:
            return self.ZONE_COLORS["end"]
        return self.ZONE_COLORS.get(zone.zone_type.value, "\033[37m")

    def _rainbow_text(self, text: str, bold: bool = False) -> str:
        """
        Render text with each character cycling through the full
        truecolor hue spectrum, rather than a fixed palette. The number
        of distinct colors used scales with len(text), so it's only
        ever bounded by terminal truecolor precision (24-bit / ~16.7M
        colors), not by any hardcoded color list.

        Args:
            text: The text to colorize character by character.
            bold: Whether to render the text in bold.

        Returns:
            ANSI-colored string, reset at the end.
        """
        n = max(len(text), 1)
        prefix = self.BOLD if bold else ""
        chars = []
        for i, ch in enumerate(text):
            hue = i / n
            r, g, b = colorsys.hsv_to_rgb(hue, 1.0, 1.0)
            ansi = (
                f"\033[38;2;{round(r * 255)};"
                f"{round(g * 255)};{round(b * 255)}m"
            )
            chars.append(f"{prefix}{ansi}{ch}")
        chars.append(self.RESET)
        return "".join(chars)

    def _colorize_text(
        self, text: str, zone: Optional[Zone], bold: bool = False
    ) -> str:
        """
        Colorize text for a given zone, handling the `rainbow` special
        case where each character gets its own color instead of one
        flat color for the whole string.

        Args:
            text: The text to colorize (usually a zone name or label).
            zone: The zone whose color/metadata should drive rendering,
                or None to fall back to dim text.
            bold: Whether to render non-rainbow text in bold.

        Returns:
            ANSI-colored string, reset at the end.
        """
        if zone is not None and zone.color == "rainbow":
            return self._rainbow_text(text, bold=bold)
        color = self._get_zone_color(zone) if zone is not None else self.DIM
        prefix = self.BOLD if bold else ""
        return f"{color}{prefix}{text}{self.RESET}"

    def _resolve_dest_zone(self, dest_label: str) -> Optional[Zone]:
        """
        Resolve a turn's destination label back to the Zone it refers to.

        Most labels are a plain zone name. A restricted-zone half-move is
        recorded instead as "zone_a-zone_b" (the link being traversed);
        since zone names can never contain a dash, that always splits
        cleanly into exactly two zone names, and the second one is the
        zone actually being entered.

        Args:
            dest_label: The destination label from a TurnRecord movement.

        Returns:
            The matching Zone, or None if it can't be resolved.
        """
        zone = self.graph.get_zone(dest_label)
        if zone is not None:
            return zone
        if '-' in dest_label:
            _, _, dest_name = dest_label.rpartition('-')
            return self.graph.get_zone(dest_name)
        return None

    def _get_drone_color(self, drone_id: int) -> str:
        """
        Get ANSI color for a drone by ID.

        Args:
            drone_id: The drone's ID.

        Returns:
            ANSI color code string.
        """
        return self.DRONE_COLORS[(drone_id - 1) % len(self.DRONE_COLORS)]

    def print_header(self) -> None:
        """Print simulation header with graph info."""
        # Ensure start_zone and end_zone are not None
        assert self.graph.start_zone is not None
        assert self.graph.end_zone is not None

        width = 60
        print()
        print(self.BOLD + "=" * width + self.RESET)
        print(self.BOLD + "  \u2708  FLY-IN" + self.RESET)
        print("=" * width)
        print(
            f"  Drones:         {self.BOLD}"
            f"{self.graph.nb_drones}{self.RESET}"
        )
        print(
            f"  Zones:          {self.BOLD}"
            f"{len(self.graph.zones)}{self.RESET}"
        )
        print(
            f"  Connections:    {self.BOLD}"
            f"{len(self.graph.connections)}{self.RESET}"
        )

        start_name = self._colorize_text(
            self.graph.start_zone.name, self.graph.start_zone, bold=True
        )
        end_name = self._colorize_text(
            self.graph.end_zone.name, self.graph.end_zone, bold=True
        )

        print(
            f"  Start:          {start_name} "
            f"({self.graph.start_zone.x},{self.graph.start_zone.y})"
        )
        print(
            f"  End:            {end_name} "
            f"({self.graph.end_zone.x},{self.graph.end_zone.y})"
        )
        print(self.BOLD + "=" * width + self.RESET)
        print()

    def print_zone_legend(self) -> None:
        """Print a legend of all zones with types and colors."""
        print(self.BOLD + "  ZONES" + self.RESET)
        for name, zone in self.graph.zones.items():
            color = self._get_zone_color(zone)
            tag = ""
            if zone.is_start:
                tag = " [START]"
            elif zone.is_end:
                tag = " [END]"
            elif zone.zone_type != ZoneType.NORMAL:
                tag = f" [{zone.zone_type.value.upper()}]"

            cap = f" cap={zone.max_drones}" if zone.max_drones > 1 else ""
            name_display = self._colorize_text(name, zone, bold=True)
            print(f"    {color}\u25cf{self.RESET} {name_display}{tag}{cap}")
        print()

    def print_turn(
        self,
        record: TurnRecord,
        drones: List[DroneState]
    ) -> None:
        """
        Print a single simulation turn with drone and zone colors.

        Each movement is rendered as "<drone>-<destination>", where the
        drone name keeps its per-drone color for readability and the
        destination is colored using that zone's own `color` metadata
        (falling back to its zone-type color if none was given, or
        rendered rainbow-per-character if the zone's color is
        `rainbow`).

        Args:
            record: The turn record with movements.
            drones: List of drone states (for color assignment).
        """
        drone_id_map: Dict[str, int] = {
            f"D{d.drone_id}": d.drone_id for d in drones
        }
        line = f"  {self.DIM}Turn {record.turn_number:3d}{self.RESET}  "
        parts: List[str] = []

        for drone_name, dest in record.movements:
            drone_id = drone_id_map.get(drone_name, 1)
            drone_color = self._get_drone_color(drone_id)

            dest_zone = self._resolve_dest_zone(dest)
            dest_display = self._colorize_text(dest, dest_zone)

            parts.append(
                f"{drone_color}{self.BOLD}{drone_name}{self.RESET}"
                f"-{dest_display}"
            )

        line += " ".join(parts)
        print(line)

    def print_simulation(
        self,
        records: List[TurnRecord],
        drones: List[DroneState]
    ) -> None:
        """
        Print the full simulation output with colors and summary.

        Args:
            records: All turn records from the simulation.
            drones: Final drone states.
        """
        self.print_header()
        self.print_zone_legend()

        print(self.BOLD + "  SIMULATION" + self.RESET)
        print()

        for record in records:
            self.print_turn(record, drones)

        print()
        print("=" * 60)
        all_arrived = all(d.arrived for d in drones)
        if all_arrived:
            print(f"  {self.BOLD}\u2713 All drones delivered!{self.RESET}")
        else:
            missing = [d.drone_name for d in drones if not d.arrived]
            print(
                f"  {self.BOLD}\u2717 Simulation stopped before all drones "
                f"arrived{self.RESET} (still en route: {', '.join(missing)})"
            )

        total_turns = records[-1].turn_number if records else 0
        print(
            f"  Total turns:            {self.BOLD}"
            f"{total_turns}{self.RESET}"
        )

        # Secondary metrics
        total_moves = sum(len(r.movements) for r in records)
        avg_per_turn = total_moves / max(total_turns, 1)
        print(
            f"  Total drone movements:  {self.BOLD}"
            f"{total_moves}{self.RESET}"
        )
        print(
            f"  Avg drones/turn:        {self.BOLD}"
            f"{avg_per_turn:.1f}{self.RESET}"
        )
        print("=" * 60)
        print()
