import math
import heapq
from drones import drones_list

class Algo:

    def __init__(self, data):
        self.data = data

        self.hubs, self.connections, self.nb_drones, self.neighbours = self.data

        self.start_hub = self.hubs['start_hub']['name']
        self.end_hub = self.hubs['end_hub']['name']

        self.unvisited = [v['name'] for v in self.hubs.values()]

    def find_the_shortest_path(self):

        short_path_dict = {}

        for e in self.unvisited:
            short_path_dict[e] = (math.inf, "none")
        short_path_dict[self.start_hub] = (0, "none")

        initiate_neighbours = list(self.neighbours[self.start_hub])
        from_hub = self.start_hub

        cost = 0

        self.unvisited.remove(self.start_hub)
        i = 0

        while initiate_neighbours:
            i +=1

            for zone in initiate_neighbours:
                if zone in self.unvisited:
                    for hub_data in self.hubs.values():
                        if hub_data['name'] == zone:
                            if hub_data['metadata']['zone'] in ["normal", "priority"]:
                                cost = 1
                                break
                            elif hub_data['metadata']['zone'] == "restricted":
                                cost = 2
                                break

                    if zone in short_path_dict:
                        if short_path_dict[zone][0] > short_path_dict[from_hub][0] + cost:
                            short_path_dict[zone] = (short_path_dict[from_hub][0] + cost, from_hub)
                    else:
                        short_path_dict[zone] = (short_path_dict[from_hub][0] + cost, from_hub)



            heap = []

            for zone, (cost, from_hub) in short_path_dict.items():
                if zone in self.unvisited:
                    heapq.heappush(heap, (cost, from_hub, zone))

            smallest_cost, from_hub, lower_cost_zone_name = heapq.heappop(heap)
            # if from_hub != self.start_hub:
            self.unvisited.remove(lower_cost_zone_name)





            # lower_cost_zone_value = math.inf
            # lower_cost_zone_name = ""

            # for zone in initiate_neighbours:
            #     if zone in self.unvisited and lower_cost_zone_value > short_path_dict[zone][0]:
            #         lower_cost_zone_value = short_path_dict[zone][0]
            #         lower_cost_zone_name = zone
            #         if from_hub != self.start_hub:
            #             self.unvisited.remove(from_hub)
            from_hub = lower_cost_zone_name
            
            if not lower_cost_zone_name or lower_cost_zone_name == self.end_hub:
                break
            else:
                initiate_neighbours = list(self.neighbours[lower_cost_zone_name])
        self.short_path = []
        a = self.end_hub
        self.short_path.append(a)
        while a != self.start_hub:
            self.short_path.append(short_path_dict[a][1])
            a = short_path_dict[a][1]
        self.short_path.reverse()
        print(self.short_path)

    def ecah_drone_path_assigner(self):

        turn = 1
        turnes = [(turn, [n['name'] for n in self.hubs.values() if n['name'] not in  [self.start_hub, self.end_hub]])]
        print(turnes)

        for drone in drones_list:
            if self.short_path[0] in turnes[turn - 1][1]:
                

