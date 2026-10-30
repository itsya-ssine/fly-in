"""
Fly-in: Drone Routing Simulation
Main entry point. Parses a map file and runs the simulation.

Usage:
    python main.py <map_file>

Arguments:
    map_file  Path to the drone network map file.
"""
import sys
import os

try:
    from parser import Parser, ParseError
    from simulation import Simulator
    from visualizer import TerminalVisualizer
except ImportError as e:
    print(f"Error: missing dependency ({e.name}).", file=sys.stderr)
    print(
        "Please run 'make install' to install all the dependencies.",
        file=sys.stderr
    )
    sys.exit(1)


class FlyInCLI:
    """
    Command-line entry point for the Fly-in drone routing simulation.

    Parses arguments, drives the parser/simulator/visualizer pipeline,
    and translates any failure along the way into an error message and
    exit code.
    """

    def __init__(self, argv: list[str]) -> None:
        """
        Args:
            argv: Full argument vector, i.e. sys.argv.
        """
        self.argv = argv

    def run(self) -> int:
        """
        Run the CLI end to end.

        Returns:
            Exit code (0 for success, 1 for error).
        """
        args = self.argv[1:]

        if len(args) != 1:
            print(
                "Usage: python main.py <map_file>",
                file=sys.stderr,
            )
            return 1

        map_file = args[0]

        if not os.path.isfile(map_file):
            print(f"Error: File not found: '{map_file}'", file=sys.stderr)
            return 1

        try:
            graph = Parser().parse_map_file(map_file)
        except ParseError as e:
            print(f"Parse error: {e}", file=sys.stderr)
            return 1
        except FileNotFoundError:
            print(f"Error: Cannot find file '{map_file}'", file=sys.stderr)
            return 1
        except PermissionError:
            print(f"Error: Cannot open file '{map_file}'", file=sys.stderr)
            return 1

        try:
            records, drones = Simulator(graph).solve()
        except RuntimeError as e:
            print(f"Simulation error: {e}", file=sys.stderr)
            return 1
        except ValueError as e:
            print(f"Simulation error: {e}", file=sys.stderr)
            return 1

        try:
            TerminalVisualizer(graph).print_simulation(records, drones)
        except (ValueError, TypeError, AttributeError) as e:
            print(f"Visualization failed: {e}", file=sys.stderr)
            return 1
        except Exception as e:
            print(f"Unexpected visualization error: {e}", file=sys.stderr)
            return 1

        return 0


if __name__ == "__main__":
    try:
        sys.exit(FlyInCLI(sys.argv).run())
    except Exception as e:
        print(f"Unknown error: {e}", sys.stderr)
