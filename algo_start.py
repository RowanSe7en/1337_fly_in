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

        self.cost = 0

    def find_the_shortest_path(self):

        self.short_path_dict = {}

        for e in self.unvisited:
            self.short_path_dict[e] = (math.inf, "none")
        from_hub = self.start_hub
        self.short_path_dict[from_hub] = (0, "none")

        initiate_neighbours = list(self.neighbours[from_hub])

        cost = 0

        self.unvisited.remove(from_hub)
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

                    if zone in self.short_path_dict:
                        if self.short_path_dict[zone][0] > self.short_path_dict[from_hub][0] + cost:
                            self.short_path_dict[zone] = (self.short_path_dict[from_hub][0] + cost, from_hub)
                    else:
                        self.short_path_dict[zone] = (self.short_path_dict[from_hub][0] + cost, from_hub)

            heap = []

            for zone, (cost, from_hub) in self.short_path_dict.items():
                if zone in self.unvisited:
                    heapq.heappush(heap, (cost, from_hub, zone))

            smallest_cost, from_hub, lower_cost_zone_name = heapq.heappop(heap)
            # if from_hub != self.start_hub:
            self.unvisited.remove(lower_cost_zone_name)

            from_hub = lower_cost_zone_name
            
            if not lower_cost_zone_name or lower_cost_zone_name == self.end_hub:
                self.cost = self.short_path_dict[self.end_hub][0]
                break
            else:
                initiate_neighbours = list(self.neighbours[lower_cost_zone_name])
        self.short_path = []
        a = self.end_hub
        self.short_path.append(a)
        while a != self.start_hub:
            self.short_path.append(self.short_path_dict[a][1])
            a = self.short_path_dict[a][1]
        self.short_path.reverse()
        self.short_path.remove(self.start_hub)
        self.short_path.remove(self.end_hub)
        # print(self.short_path)

    def ecah_drone_path_assigner(self):

        turnes = [(1, [n['name'] for n in self.hubs.values() if n['name'] not in [self.start_hub]])]

        for drone in drones_list:
            # print('a')
            turn = 0
            cost = 0
            for zone in self.short_path:
                # print("b")
                turn += 1
                if len(turnes) >= turn:
                    if zone in turnes[turn - 1][1]:
                        for k, v in self.hubs.items():
                            if v['name'] == zone:
                                zone_name = v['metadata']['zone']
                        if zone_name == "restricted":
                            stay = 2
                        elif zone_name == "normal":
                            stay = 1
                        for i in range(stay):
                            drone.path.append(zone)
                            if i >= 1:
                                turn += 1
                                turnes.append((turn, [n['name'] for n in self.hubs.values() if n['name'] not in  [self.start_hub]]))
                            turnes[turn - 1][1].remove(zone)
                    else:
                        print("else")
                        added_cost = 0
                        for t in turnes:
                            if zone not in t[1]:
                                added_cost += 1
                            else:
                                break
                        print(cost)

                        self.short_path_dict_2 = {}

                        self.unvisited2 = [v['name'] for v in self.hubs.values()]

                        for e in self.unvisited2:
                            self.short_path_dict_2[e] = (math.inf, "none")
                        from_hub2 = self.short_path_dict[zone][1] # this is correct
                        sstart_hub = from_hub2
                        self.short_path_dict_2[from_hub2] = (0, self.short_path_dict[from_hub2][1])

                        initiate_neighbours = list(self.neighbours[from_hub2])

                        cost = 0

                        self.unvisited2.remove(from_hub2)
                        i = 0

                        while initiate_neighbours:
                            
                            i += 1

                            for alt_zone in initiate_neighbours:
                                if alt_zone in self.unvisited2 and alt_zone not in [zone, self.short_path_dict[zone][1]]:
                                    # print("g", alt_zone)
                                    for hub_data in self.hubs.values():
                                        if hub_data['name'] == alt_zone:
                                            if hub_data['metadata']['zone'] in ["normal", "priority"]:
                                                cost = 1
                                                break
                                            elif hub_data['metadata']['zone'] == "restricted":
                                                cost = 2
                                                break

                                    if alt_zone in self.short_path_dict_2 and alt_zone in turnes[turn - 1][1]:
                                        if self.short_path_dict_2[alt_zone][0] > self.short_path_dict_2[from_hub2][0] + cost:
                                            self.short_path_dict_2[alt_zone] = (self.short_path_dict_2[from_hub2][0] + cost, from_hub2)
                                    else:
                                        if alt_zone in turnes[turn - 1][1]:
                                            self.short_path_dict_2[alt_zone] = (self.short_path_dict_2[from_hub2][0] + cost, from_hub2)
                                # print(self.short_path_dict_2)

                            heap = []

                            for alt_zone, (cost, from_hub2) in self.short_path_dict_2.items():
                                if alt_zone in self.unvisited2:
                                    heapq.heappush(heap, (cost, from_hub2, alt_zone))

                            # print("gggggggggggg", heap)

                            smallest_cost, from_hub2, lower_cost_zone_name = heapq.heappop(heap)
                            # if from_hub2 != self.start_hub:
                            self.unvisited2.remove(lower_cost_zone_name)

                            from_hub2 = lower_cost_zone_name
                            
                            if not lower_cost_zone_name or lower_cost_zone_name == self.end_hub:
                                # print(self.short_path_dict_2)
                                break
                            else:
                                initiate_neighbours = list(self.neighbours[lower_cost_zone_name])
                        self.short_path2 = []
                        print(self.short_path_dict_2)
                        a = self.end_hub
                        self.short_path2.append(a)
                        while a != sstart_hub:
                            self.short_path2.append(self.short_path_dict_2[a][1])
                            a = self.short_path_dict_2[a][1]
                        self.short_path2.reverse()
                        self.short_path2.remove(sstart_hub)
                        self.short_path2.remove(self.end_hub)
                        print("ewferrgterr", self.short_path2)
                        print("cost", self.short_path_dict_2[self.end_hub][0])

                        if self.short_path_dict_2[self.end_hub][0] <= self.cost + added_cost:
                            pass












                else:
                    turnes.append((turn, [n['name'] for n in self.hubs.values() if n['name'] not in  [self.start_hub]]))
                    drone.path.append(zone)
                    turnes[turn - 1][1].remove(zone)
                # print(turnes)
            print("ttttt", turnes)
            print("ddddd", drone.path)
            print("-------------------------------------")
