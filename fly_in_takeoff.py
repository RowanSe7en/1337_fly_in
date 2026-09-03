import argparse
from pathlib import Path
from typing import Dict, List, Tuple
from algo_start import Algo
from custom_types import ParsedData
from display import Display
from drones import drone_creator, drones_list
from parsing import Parser
from simulation_printer import SimulationPrinter
from zone_reachability import ZoneReachability


def main() -> None:
    """Run the Fly-in drone simulation.

    Parses the map file provided through the command-line arguments,
    validates zone reachability, calculates drone paths, runs the
    simulation, and displays the resulting movements.

    Raises:
        FileNotFoundError: If the specified map file does not exist.
        PermissionError: If the specified map file cannot be accessed.
        ValueError: If the map contains invalid values.
        TypeError: If the map contains invalid data types or structure.
        OSError: If another operating-system-level error occurs.
    """
    arg_parser: argparse.ArgumentParser = argparse.ArgumentParser(
        description="Run the Fly-in drone simulation."
    )

    arg_parser.add_argument(
        "-m",
        "--map",
        type=Path,
        default=Path("maps/easy/mine.txt"),
        help="Path to the map file.",
    )

    args: argparse.Namespace = arg_parser.parse_args()

    try:
        data: ParsedData = Parser(args.map).parse()

        ZoneReachability(data).check_all_zones_reachable()

        algo: Algo = Algo(data)
        algo.find_the_shortest_path()

        drone_creator(algo.nb_drones)

        zones_at_turnes: List[Tuple[int, Dict[str, List[int]]]] = (
            algo.ecah_drone_path_assigner()
        )

        visual: SimulationPrinter = SimulationPrinter(
            zones_at_turnes,
            drones_list,
            algo.end_hub,
            algo.get_zone_type,
        )

        moves: int = visual.print_simulation()

        display: Display = Display(data, zones_at_turnes, moves)
        display.run()

    except FileNotFoundError:
        print(f"<<ERROR DETECTED>>: Map file not found: {args.map}")

    except PermissionError:
        print(f"<<ERROR DETECTED>>: Permission denied: {args.map}")

    except (ValueError, TypeError) as error:
        print(f"<<ERROR DETECTED>>: {error}")

    except OSError as error:
        print(f"<<ERROR DETECTED>>: {error}")


if __name__ == "__main__":
    main()
