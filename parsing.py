import math


class Parser:

    VALID_FIELDS = ["nb_drones", "start_hub", "hub", "end_hub", "connection"]
    VALID_ZONES = ["normal", "blocked", "restricted", "priority"]

    def __init__(self, path):

        self.path = path

        self.hubs = {}
        self.hub_names = {}
        self.connections = {}
        self.neighbours = {}

        self.fields = []
        self.hub_coords = []
        self.edges = []
        self.sorted_edges = []

        self.nb_drones = 1
        self.is_first_line = True

        self.hub_idx = 1
        self.connection_idx = 1

    def parse(self):

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
                    self._parse_hub(field, content)

                elif field == "connection":
                    self._parse_connection(content)

        self._validate()

        return self.hubs, self.connections, self.nb_drones, self.neighbours

    def _parse_hub(self, field, content):
        parts = content.split()

        hub_name = parts[0]

        if "-" in hub_name or " " in hub_name:
            raise TypeError("Hub names cannot contain dash '-' or space ' '")

        hub_data = {
            "name": hub_name,
            "x": int(parts[1]),
            "y": int(parts[2]),
        }

        if hub_name in self.hub_names:
            raise TypeError("Hub names should be unique")

        self.hub_names[hub_name] = 0

        coords = (hub_data["x"], hub_data["y"])

        if coords in self.hub_coords:
            raise TypeError("Hub coords should be unique")

        self.hub_coords.append(coords)

        hub_metadata = {}

        start = content.find("[")
        end = content.find("]")

        if start != -1 and end != -1:
            for item in content[start + 1:end].split():
                meta_key, meta_value = item.split("=")

                if meta_key == "max_drones":
                    meta_value = int(meta_value)

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

    def _parse_connection(self, content):
        connection_data = {}
        connection_metadata = {}

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

        if from_hub not in self.hub_names or to_hub not in self.hub_names:
            raise TypeError("Edge name not found, or not declared yet")

        start = content.find("[")
        end = content.find("]")

        if start != -1 and end != -1:
            for item in content[start + 1:end].split():
                meta_key, meta_value = item.split("=", 1)

                if meta_key == "max_link_capacity":
                    meta_value = int(meta_value)

                    if meta_value <= 0:
                        print(
                            "max_link_capacity should be at least 1 or more, "
                            "falling back to the default value 1"
                        )
                        meta_value = 1

                connection_metadata[meta_key] = meta_value

        connection_metadata.setdefault("max_link_capacity", 1)

        connection_data["from_hub"] = from_hub
        connection_data["to_hub"] = to_hub
        connection_data["metadata"] = connection_metadata

        self.connections[f"connection{self.connection_idx}"] = connection_data
        self.connection_idx += 1

        self.hub_names[from_hub] += 1
        self.hub_names[to_hub] += 1

        self.edges.append((from_hub, to_hub))

    def _validate(self):
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

    def print_data(self):

        print(f"nb_drones: {self.nb_drones}")

        print("-------------------")
        for hub_name, hub_data in self.hubs.items():
            print(f"{hub_name} = {hub_data}")

        print("-------------------")
        for connection_name, connection_data in self.connections.items():
            print(f"{connection_name} = {connection_data}")
