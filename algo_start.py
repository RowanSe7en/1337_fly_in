import math

class Algo:

    def __init__(self, data):
        self.data = data

        self.hubs, self.connections, self.nb_drones, self.neighbours = self.data

        start_hub = self.hubs['start_hub']['name']

        self.unvisited = [v['name'] for v in self.hubs.values()]

        short_path_dict = {}

        for e in self.unvisited:
            short_path_dict[e] = (math.inf, "none")
        short_path_dict[start_hub] = (0, "none")

        print(self.neighbours)