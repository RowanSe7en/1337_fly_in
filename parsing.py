import math
from pathlib import Path
from typing import Any, Dict, List, Tuple, Union
from custom_types import ConnectionDict, HubDict, ParsedData


class Parser:
    """Parse and validate the Fly-in simulation input file.

    The parser reads hub, connection, and drone information from an input
    file, validates the provided data, and builds the structures required
    by the simulation.
    """

    VALID_FIELDS: List[str] = [
        "nb_drones",
        "start_hub",
        "hub",
        "end_hub",
        "connection",
    ]
    VALID_ZONES: List[str] = ["normal", "blocked", "restricted", "priority"]

    def __init__(self, path: Union[str, Path]) -> None:
        """Initialize the parser with an input file path.

        Args:
            path: Path to the input file containing the simulation data.
        """
        self.path: Union[str, Path] = path
        self.hubs: Dict[str, HubDict] = {}
        self.hub_names: Dict[str, int] = {}
        self.connections: Dict[str, ConnectionDict] = {}
        self.neighbours: Dict[str, List[str]] = {}
        self.fields: List[str] = []
        self.hub_coords: List[Tuple[int, int]] = []
        self.edges: List[Tuple[str, str]] = []
        self.sorted_edges: List[List[str]] = []
        self.nb_drones: int = 1
        self.is_first_line: bool = True
        self.hub_idx: int = 1
        self.connection_idx: int = 1

    def parse(self) -> ParsedData:
        """Parse the input file and return the simulation data.

        Reads each valid line from the input file, parses hubs and
        connections, validates the resulting data, and builds the neighbour
        relationships between hubs.

        Returns:
            tuple: A tuple containing the hubs, connections, number of
            drones, and neighbour mappings.

        Raises:
            TypeError: If the input contains invalid fields, invalid field
            ordering, or malformed hub or connection data.
        """

        with open(self.path, "r") as file:

            while True:

                line = file.readline()

                if not line:
                    break

                if "#" in line or not line.strip():
                    continue

                field, content = line.split(":")
                field = field.strip()
                content = content.strip()

                if not field:
                    raise TypeError(
                        f"You didnt pass a field name {Parser.VALID_FIELDS}"
                    )

                if field not in self.fields:
                    self.fields.append(field)
                elif field not in ("hub", "connection"):
                    print(
                        f"Duplicated {field} detected, the field will"
                        " be overwritten and the program will continue"
                    )

                if field == "nb_drones" and self.is_first_line:
                    self.nb_drones = int(content)

                    if self.nb_drones <= 0:
                        print(
                            "nb_drones should be at least 1 or more, "
                            "falling back to the default value 1"
                        )
                        self.nb_drones = 1

                    self.is_first_line = False
                    continue

                elif self.is_first_line:
                    raise TypeError("nb_drones should be the first field")

                if field in ("start_hub", "hub", "end_hub"):
                    self.parse_hub(field, content)
                elif field == "connection":
                    self.parse_connection(content)

        self.validate()

        return (
            self.hubs,
            self.connections,
            self.nb_drones,
            self.neighbours,
        )

    def parse_hub(self, field: str, content: str) -> None:
        """Parse and store a hub definition.

        Extracts the hub name, coordinates, and optional metadata from the
        provided content. The method validates hub names and coordinates,
        applies default metadata values, and stores the hub in the parser's
        hub collection.

        Args:
            field: Type of hub being parsed, such as ``start_hub``,
                ``hub``, or ``end_hub``.
            content: Raw hub definition containing the name, coordinates,
                and optional metadata.

        Raises:
            TypeError: If the hub name contains invalid characters, a hub
                name is duplicated, coordinates are duplicated, or an
                invalid zone type is provided.
        """
        parts = content.split()
        hub_name: str = parts[0]

        if "-" in hub_name or " " in hub_name:
            raise TypeError("Hub names cannot contain dash '-' or space ' '")

        hub_data: HubDict = {
            "name": hub_name,
            "x": int(parts[1]),
            "y": int(parts[2]),
        }

        if hub_name in self.hub_names:
            raise TypeError("Hub names should be unique")

        self.hub_names[hub_name] = 0

        coords: Tuple[int, int] = (hub_data["x"], hub_data["y"])

        if coords in self.hub_coords:
            raise TypeError("Hub coords should be unique")

        self.hub_coords.append(coords)

        hub_metadata: Dict[str, Any] = {}
        start = content.find("[")
        end = content.find("]")

        if start != -1 and end != -1:
            meta_value: Union[str, int]

            for item in content[start + 1:end].split():
                meta_key, raw_value = item.split("=")
                meta_value = raw_value

                if meta_key == "max_drones":
                    meta_value = int(raw_value)

                    if meta_value <= 0:
                        print(
                            "max_drones should be at least 1 or more, "
                            "falling back to the default value 1"
                        )
                        meta_value = 1

                elif meta_key == "zone":
                    if meta_value not in self.VALID_ZONES:
                        raise TypeError(
                            "Zone type not recognized, "
                            f"use one of these {self.VALID_ZONES}"
                        )

                hub_metadata[meta_key] = meta_value

        hub_metadata.setdefault("zone", "normal")
        hub_metadata.setdefault("color", "none")
        hub_metadata.setdefault("max_drones", 1)

        if field in ("start_hub", "end_hub"):
            hub_metadata["max_drones"] = math.inf

        hub_data["metadata"] = hub_metadata

        if field == "hub":
            self.hubs[f"hub{self.hub_idx}"] = hub_data
            self.hub_idx += 1
        else:
            self.hubs[field] = hub_data

    def parse_connection(self, content: str) -> None:
        """Parse and store a connection between two hubs.

        Extracts the connected hub names and optional connection metadata,
        updates the neighbour relationships, and stores the connection in
        the parser's connection collection.

        Args:
            content: Raw connection definition containing the connected
                hubs and optional metadata.

        Raises:
            TypeError: If the connection format is invalid or one of the
                referenced hubs has not been declared.
        """
        connection_data: ConnectionDict = {}
        connection_metadata: Dict[str, Any] = {}

        edge = content.split()[0]

        if edge.count("-") != 1:
            raise TypeError(
                "connection names cannot contain space ' ' or dash '-', "
                "instead they should be separated by one"
            )

        from_hub, to_hub = edge.split("-")

        if from_hub in self.neighbours:
            self.neighbours[from_hub].append(to_hub)
        else:
            self.neighbours[from_hub] = [to_hub]

        if to_hub in self.neighbours:
            self.neighbours[to_hub].append(from_hub)
        else:
            self.neighbours[to_hub] = [from_hub]

        if (
            from_hub not in self.hub_names
            or to_hub not in self.hub_names
        ):
            raise TypeError("Edge name not found, or not declared yet")

        start = content.find("[")
        end = content.find("]")

        if start != -1 and end != -1:
            meta_value: Union[str, int]

            for item in content[start + 1:end].split():
                meta_key, raw_value = item.split("=", 1)
                meta_value = raw_value

                if meta_key == "max_link_capacity":
                    meta_value = int(raw_value)

                    if meta_value <= 0:
                        print(
                            "max_link_capacity should be at least 1 or "
                            "more, falling back to the default value 1"
                        )
                        meta_value = 1

                connection_metadata[meta_key] = meta_value

        connection_metadata.setdefault("max_link_capacity", 1)

        connection_data["from_hub"] = from_hub
        connection_data["to_hub"] = to_hub
        connection_data["metadata"] = connection_metadata

        self.connections[
            f"connection{self.connection_idx}"
        ] = connection_data

        self.connection_idx += 1
        self.hub_names[from_hub] += 1
        self.hub_names[to_hub] += 1
        self.edges.append((from_hub, to_hub))

    def validate(self) -> None:
        """Validate the parsed fields, hubs, and connections.

        Ensures that all required field types are present and verifies that
        duplicate connections do not exist. A connection is considered a
        duplicate regardless of the order of its two hubs.

        Raises:
            TypeError: If required fields are missing or the same connection
                appears more than once.
        """
        if sorted(self.VALID_FIELDS) != sorted(self.fields):
            raise TypeError(
                f"You missed one of these fields {self.VALID_FIELDS}"
            )

        for hub_name, count in self.hub_names.items():
            if count == 0:
                print(
                    f"Pay attention the zone {hub_name} is not used, "
                    "but the program will continue"
                )

        self.sorted_edges = [sorted(edge) for edge in self.edges]

        for edge in self.sorted_edges:
            if self.sorted_edges.count(edge) > 1:
                raise TypeError(
                    "The same connection must not appear more than once "
                    "(e.g., a-b and b-a are considered duplicates)"
                )

    def print_data(self) -> None:
        """Print the parsed drone, hub, and connection data.

        Displays the number of drones followed by the parsed hub and
        connection dictionaries.

        Returns:
            None: This method prints the parsed data to standard output.
        """
        print(f"nb_drones: {self.nb_drones}")
        print("-------------------")

        for hub_name, hub_data in self.hubs.items():
            print(f"{hub_name} = {hub_data}")

        print("-------------------")

        for connection_name, connection_data in self.connections.items():
            print(f"{connection_name} = {connection_data}")
