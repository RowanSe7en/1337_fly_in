import argparse
from pathlib import Path

from algo_start import Algo
from display import Display
from drones import drone_creator, drones_list
from parsing import Parser
from simulation_printer import SimulationPrinter
from zone_reachability import ZoneReachability


def main():
    """Run the Fly-in simulation."""

    arg_parser = argparse.ArgumentParser(
        description="Run the Fly-in drone simulation."
    )

    arg_parser.add_argument(
        "-m",
        "--map",
        type=Path,
        default=Path("maps/easy/mine.txt"),
        help="Path to the map file.",
    )

    args = arg_parser.parse_args()

    try:

        data = Parser(args.map).parse()
        ZoneReachability(data).check_all_zones_reachable()

        algo = Algo(data)
        algo.find_the_shortest_path()

        drone_creator(algo.nb_drones)
        zones_at_turnes = algo.ecah_drone_path_assigner()

        visual = SimulationPrinter(
            zones_at_turnes,
            drones_list,
            algo.end_hub,
            algo.get_zone_type,
        )
        moves = visual.print_simulation()

        display = Display(data, zones_at_turnes, moves)
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
