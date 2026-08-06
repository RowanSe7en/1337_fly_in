
import math

from colors import COMMON_COLORS

def start(path):
    

    with open(path, "r") as file:
        hubs = {}
        hubs_names = {}
        fields = []
        valid_fields = ["nb_drones", "start_hub", "hub", "end_hub", "connection"]
        valid_zones = ["normal", "blocked", "restricted", "priority"]
        hubs_cords = []
        edges = []
        sorted_edges = []
        connections = {}
        nb_drones = 1
        is_first_line = True
        i = 1
        j = 1
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
                raise TypeError("You didnt pass a field name ('nb_drones', 'start_hub', 'hub', 'end_hub', 'connection')")
            if field not in fields:
                fields.append(field)
            elif field not in ("hub", "connection"):
                print(f"Duplicated {field} detected, the field will be overritten and the program will continue")
            if field == "nb_drones" and is_first_line:
                nb_drones = int(content)
                if nb_drones <= 0:
                    print("nb_drones should be at least 1 or more, falling back to the default value 1")
                    nb_drones = 1
                is_first_line = False
                continue
            elif is_first_line:
                raise TypeError("nb_drones should be the first field")

            if field in ("start_hub", "hub", "end_hub"):
                parts = content.split()

                hub_name = parts[0]
                if '-' in hub_name or ' ' in hub_name:
                    raise TypeError("Hub names cannot contain dash '-' or space ' '")

                hub_data = {
                    "name": hub_name,
                    "x": int(parts[1]),
                    "y": int(parts[2])
                }

                if hub_name in hubs_names.keys():
                    raise TypeError("Hub names should be unique")
                else:
                    hubs_names[hub_name] = 0

                if (hub_data['x'], hub_data['y']) in hubs_cords or hub_data['x'] < 0 or hub_data['y'] < 0:
                    raise TypeError("Hub coords should be unique and positive")
                else:
                    hubs_cords.append((hub_data['x'], hub_data['y']))

                hub_metadata = {}

                start = content.find("[")
                end = content.find("]")

                if start != -1 and end != -1:
                    for item in content[start + 1:end].split():
                        meta_key, meta_value = item.split("=")
                        if meta_key == "max_drones":
                            meta_value = int(meta_value)
                            if meta_value <= 0:
                                print("max_drones should be at least 1 or more, falling back to the default value 1")
                                meta_value = 1
                        elif meta_key == "zone" and meta_value not in valid_zones:
                            raise TypeError(f"Zone type not recognized, use one of these {valid_zones}")
                        hub_metadata[meta_key] = meta_value

                hub_metadata.setdefault("zone", "normal")
                hub_metadata.setdefault("color", "none")
                hub_metadata.setdefault("max_drones", 1)

                if field in ["start_hub", "end_hub"]:
                   hub_metadata["max_drones"] = math.inf

                if hub_metadata['color'] not in COMMON_COLORS:
                    raise TypeError(f"Color not recognized, use one of these {COMMON_COLORS}")

                hub_data["metadata"] = hub_metadata
                if field == "hub":
                    hubs[f"{field}{i}"] = hub_data
                    i += 1
                else:
                    hubs[field] = hub_data

            elif field == "connection":
                connection_data = {}
                connection_metadata = {}

                edge = content.split()[0]

                if edge.count("-") != 1:
                    raise TypeError("connection names cannot contain space ' ' or dash '-', insted they should be seperated by one")

                from_hub, to_hub = edge.split("-")

                if from_hub not in hubs_names.keys() or to_hub not in hubs_names.keys():
                    raise TypeError("Edge name not found, or not declared yet")

                start = content.find("[")
                end = content.find("]")

                if start != -1 and end != -1:
                    for item in content[start + 1:end].split():
                        meta_key, meta_value = item.split("=", 1)
                        
                        if meta_key == "max_link_capacity":
                            connection_metadata[meta_key] = int(meta_value)
                            if int(meta_value) <= 0:
                                print("max_link_capacity should be at least 1 or more, falling back to the default value 1")
                                meta_value = 1
                        connection_metadata[meta_key] = meta_value

                connection_metadata.setdefault("max_link_capacity", 1)

                connection_data["from_hub"] = from_hub
                connection_data["to_hub"] = to_hub
                connection_data["metadata"] = connection_metadata

                connections[f"{field}{j}"] = connection_data
                j += 1

                hubs_names[from_hub] += 1
                hubs_names[to_hub] += 1

                edges.append((from_hub, to_hub))
        if sorted(valid_fields) != sorted(fields):
            raise TypeError(f"You missed on of these fields {valid_fields}")
        for key, value in hubs_names.items():
            if not value:
                print(f"Pay attention the zone {key} is not used, but the program will continue")
        for e in edges:
            sorted_edges.append(sorted(e))
        for e in sorted_edges:
            if sorted_edges.count(e) > 1:
                raise TypeError("The same connection must not appear more than once (e.g., a-b and b-a are considered duplicates)")
    print("-------------------")
    for hub_name, hub_data in hubs.items():
        print(f"{hub_name} = {hub_data}")

    print("-------------------")
    for from_hub, connection_data in connections.items():
        print(f"{from_hub} = {connection_data}")
