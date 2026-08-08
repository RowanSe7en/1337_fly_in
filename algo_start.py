import math
import heapq

class Algo:

    def __init__(self, data):
        self.data = data

        self.hubs, self.connections, self.nb_drones, self.neighbours = self.data

        start_hub = self.hubs['start_hub']['name']

        self.unvisited = [v['name'] for v in self.hubs.values()]

        print(self.unvisited)

        short_path_dict = {}

        for e in self.unvisited:
            short_path_dict[e] = (math.inf, "none")
        short_path_dict[start_hub] = (0, "none")

        # print(short_path_dict)
        print(self.neighbours)

        initiate_neighbours = list(self.neighbours[start_hub])
        from_hub = start_hub

        print(initiate_neighbours)

        cost = 0

        self.unvisited.remove(start_hub)
        # print("ggggggggggggggggggggggggggggggggg")
        i = 0

        while initiate_neighbours:
            i +=1

            for zone in initiate_neighbours:
                if zone in self.unvisited:
                    # print(f"A: {i} {self.unvisited} {zone}")
                    # print(self.hubs)
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
            # if from_hub != start_hub:
            self.unvisited.remove(lower_cost_zone_name)
            print(f"{i} {lower_cost_zone_name}")



            # print("tttttttttttttt")


            # lower_cost_zone_value = math.inf
            # lower_cost_zone_name = ""

            # for zone in initiate_neighbours:
            #     if zone in self.unvisited and lower_cost_zone_value > short_path_dict[zone][0]:
            #         # print(f"B: {i} {self.unvisited} {zone}")
            #         lower_cost_zone_value = short_path_dict[zone][0]
            #         lower_cost_zone_name = zone
            #         # print(f"C: {i} {zone}")
            #         if from_hub != start_hub:
            #             self.unvisited.remove(from_hub)
            from_hub = lower_cost_zone_name
            
            if not lower_cost_zone_name or lower_cost_zone_name == self.hubs['end_hub']['name']:
                break
            else:
                initiate_neighbours = list(self.neighbours[lower_cost_zone_name])
        print(short_path_dict)
        short_path = []
        a = self.hubs['end_hub']['name']
        short_path.append(a)
        while a != start_hub:
            short_path.append(short_path_dict[a][1])
            a = short_path_dict[a][1]
        short_path.reverse()
        print(short_path)
