import sys
from pathlib import Path

pa = Path(sys.argv[1])

with open(pa, 'r') as file:

    nodes_dict = {}
    connections_dict = {}
    nb_drones = 1
    is_first_line = True


    while True:

        my_line = file.readline()

        if not my_line:
            break
        if '#' in my_line or not my_line.strip():
            continue
        key = my_line.split(':')[0].strip()
        value = my_line.split(':')[1].strip()
        if key == "nb_drones" and is_first_line:
            nb_drones = value
            is_first_line = False
        elif key == "nb_drones" and not is_first_line:
            raise ValueError("nb_drones should be the first filed")
        if key in ["start_hub", "hub", "end_hub"]:
            key = value.strip().split(' ')[0]
            start = value.find('[')
            end = value.find(']')
            new_value = value.strip().split(' ')[1::]
            metadata_dict = {}
            nodedata_dict = {}
            nodedata_dict['x'] = int(new_value[0])
            nodedata_dict['y'] = int(new_value[1])
            if new_value[2]:
                bracket_data = value[start + 1: end].split(' ')
                for e in bracket_data:
                    metakey = e.strip().split('=')[0]
                    metavalue = e.strip().split('=')[1]
                    metadata_dict[metakey] = metavalue
                    nodedata_dict['metadata'] = metadata_dict
            nodes_dict[key] = nodedata_dict
            if not nodedata_dict['metadata'].get("zone", 0):
                nodedata_dict['metadata']["zone"] = "normal"
            if not nodedata_dict['metadata'].get("color", 0):
                nodedata_dict['metadata']["color"] = "none"            
            if not nodedata_dict['metadata'].get("max_drones", 0):
                nodedata_dict['metadata']["max_drones"] = 1
        elif key == "connection":
            bidirectional = value.strip().split(' ')[0]
            start = value.find('[')
            end = value.find(']')
            max_link_capacity = value[start + 1: end].split('=')[1]
            if bidirectional.count('-') > 1:
                raise ValueError("name cannot have dash in it")
            key = bidirectional.strip().split('-')[0]
            value_dict = {} 
            value_dict['to'] = bidirectional.strip().split('-')[1]
            value_dict['max_link_capacity'] = max_link_capacity
            connections_dict[key] = value_dict

print(nb_drones)

for key, value in nodes_dict.items():
    print(f"{key} = {value}")
print("-------------------")
for key, value in connections_dict.items():
    print(f"{key} = {value}")