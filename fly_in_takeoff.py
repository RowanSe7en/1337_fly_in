import sys
from pathlib import Path
from parsing import Parser
from algo_start import Algo


def main():

    path = Path(sys.argv[1])

    try:
        parser = Parser(path)
        data = parser.parse()
        # parser.print_data()

        algo = Algo(data)

    except ValueError:
        print(
            "You probably passed non-integers where integers are expected"
            ", or the file format is invalid."
            )

    except TypeError as error:
        print(f"<<ERROR DETECTED>>: {error}")

    # except Exception:
    #     print("Misstructured file format.")


if __name__ == "__main__":
    main()
