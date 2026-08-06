from parsing import start
import sys
from pathlib import Path

def main():

    path = Path(sys.argv[1])

    try:
        start(path)
    except ValueError as error:
        print("You propabaly passed non-integers where you should pass integer, or misstructured the format")
    except TypeError as error:
        print(f"<<ERROR DETECTED>>: {error}")
    except Exception as error:
        print("misstructured the format")

if __name__ == "__main__":
    main()