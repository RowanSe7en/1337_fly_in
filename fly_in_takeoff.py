import argparse
from pathlib import Path

from parsing import Parser
from algo_start import Algo
from drones import drone_creator
from display import Display


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

    path = args.map

    try:
        parser = Parser(path)
        data = parser.parse()

    except ValueError:
        print(
            "You probably passed non-integers where integers are expected"
            ", or the file format is invalid."
            )

    except TypeError as error:
        print(f"<<ERROR DETECTED>>: {error}")

    # except Exception:
    #     print("Misstructured file format.")








    algo = Algo(data)
    algo.find_the_shortest_path()

    drone_creator(algo.nb_drones)
    zones_at_turnes = algo.ecah_drone_path_assigner()
    algo.print_simulation()

    display = Display(data, zones_at_turnes)
    display.run()


    # try:
    #     algo = Algo(data)
    #     algo.find_the_shortest_path()

    #     drone_creator(algo.nb_drones)
    #     algo.ecah_drone_path_assigner()

    
    # except Exception as error:
    #     print(f"<<ERROR DETECTED>>: {error}")

if __name__ == "__main__":
    main()
