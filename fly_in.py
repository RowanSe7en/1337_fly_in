import sys
from pathlib import Path
from colors import COMMON_COLORS

def start():
    path = Path(sys.argv[1])

    with open(path, "r") as file:
        hubs = {}
        hubs_names = []
        fields = []
        valid_fields = ["nb_drones", "start_hub", "hub", "end_hub", "connection"]
        hubs_cords = []
        connections = {}
        nb_drones = 1
        is_first_line = True
        i = 1
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
            if field == "nb_drones" and is_first_line:
                nb_drones = int(content)
                is_first_line = False
                continue
            elif is_first_line:
                raise TypeError("nb_drones should be the first field")

            if field in ("start_hub", "hub", "end_hub"):
                parts = content.split()

                hub_name = parts[0]
                if '-' in hub_name:
                    raise TypeError("Hub names cannot contain '-'")

                hub_data = {
                    "name": hub_name,
                    "x": int(parts[1]),
                    "y": int(parts[2])
                }

                if hub_name in hubs_names:
                    raise TypeError("Hub names should be unique")
                else:
                    hubs_names.append(hub_name)

                if (hub_data['x'], hub_data['y']) in hubs_cords:
                    raise TypeError("Hub coords should be unique")
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
                        hub_metadata[meta_key] = meta_value

                hub_metadata.setdefault("zone", "normal")
                hub_metadata.setdefault("color", "none")
                hub_metadata.setdefault("max_drones", 1)

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
                    raise TypeError("connection names cannot contain '-', insted they should be seperated by one")

                from_hub, to_hub = edge.split("-")

                start = content.find("[")
                end = content.find("]")

                if start != -1 and end != -1:
                    for item in content[start + 1:end].split():
                        meta_key, meta_value = item.split("=", 1)
                        
                        if meta_key == "max_link_capacity":
                            connection_metadata[meta_key] = int(meta_value)
                        connection_metadata[meta_key] = meta_value

                connection_metadata.setdefault("max_link_capacity", 1)

                connection_data["to_hub"] = to_hub
                connection_data["metadata"] = connection_metadata

                connections[from_hub] = connection_data
        if sorted(valid_fields) != sorted(fields):
            raise TypeError(f"You missed on of these fields {valid_fields}")
    print("-------------------")
    for hub_name, hub_data in hubs.items():
        print(f"{hub_name} = {hub_data}")

    print("-------------------")
    for from_hub, connection_data in connections.items():
        print(f"{from_hub} = {connection_data}")

def main():

    try:
        start()
    except ValueError as error:
        print("You propabaly passed non-integers where you should pass integer, or misstructured the format")
    except TypeError as error:
        print(error)
    except Exception as error:
        print("misstructured the format")
if __name__ == "__main__":
    main()