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
        self.all_paths = []

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
                is_blocked = 0
                if zone in self.unvisited:
                    for hub_data in self.hubs.values():
                        if hub_data['name'] == zone:
                            if hub_data['metadata']['zone'] == "blocked":
                                print("blocked")
                                is_blocked = 1
                                break
                            elif hub_data['metadata']['zone'] in ["normal", "priority"]:
                                cost = 1
                                break
                            elif hub_data['metadata']['zone'] == "restricted":
                                cost = 2
                                break

                    if is_blocked:
                        continue

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
        print(self.short_path)

    def ecah_drone_path_assigner(self):

        turnes = [(1, [n['name'] for n in self.hubs.values() if n['name'] not in [self.start_hub]])]

        for drone in drones_list:
            print("drone.id", drone.id)
            turn = 0
            cost = 0
            for zone in self.short_path:
                turn += 1
                if len(turnes) >= turn:
                    print("qqqqqqq", turn)
                    if zone in turnes[turn - 1][1]:
                        drone.start_turn = turn
                        print("hi1")
                        for k, v in self.hubs.items():
                            if v['name'] == zone:
                                print("zone", zone)
                                zone_name = v['metadata']['zone']
                        if zone_name == "restricted":
                            stay = 2
                        else:
                            stay = 1
                        for i in range(stay):
                            drone.path.append(zone)
                            if i >= 1:
                                turn += 1
                                turnes.append((turn, [n['name'] for n in self.hubs.values() if n['name'] not in  [self.start_hub]]))
                            turnes[turn - 1][1].remove(zone)
                        print("drone.path gggggg", drone.path)
                    else:
                        is_found = 0
                        for e in self.neighbours[self.short_path_dict[zone][1]]:
                            for k, v in self.hubs.items():
                                if v['name'] == zone:
                                    zone_name = v['metadata']['zone']
                            if e in turnes[turn - 1][1] and zone_name != "blocked":
                                is_found = 1
                                break
                        if is_found:
                            print("else")
                            added_cost = 0
                            for t in turnes:
                                if zone not in t[1]:
                                    added_cost += 1
                                else:
                                    break
                            print("cost", self.cost)
                            print("added_cost", added_cost)

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

                                heap = []

                                for alt_zone, (cost, from_hub2) in self.short_path_dict_2.items():
                                    if alt_zone in self.unvisited2:
                                        heapq.heappush(heap, (cost, from_hub2, alt_zone))


                                smallest_cost, from_hub2, lower_cost_zone_name = heapq.heappop(heap)
                                # if from_hub2 != self.start_hub:
                                self.unvisited2.remove(lower_cost_zone_name)

                                from_hub2 = lower_cost_zone_name
                                
                                if not lower_cost_zone_name or lower_cost_zone_name == self.end_hub:
                                    break
                                else:
                                    initiate_neighbours = list(self.neighbours[lower_cost_zone_name])
                            self.short_path2 = []
                            a = self.end_hub
                            self.short_path2.append(a)
                            while a != sstart_hub:
                                self.short_path2.append(self.short_path_dict_2[a][1])
                                a = self.short_path_dict_2[a][1]
                            self.short_path2.reverse()
                            self.short_path2.remove(sstart_hub)
                            self.short_path2.remove(self.end_hub)
                            new_cost = self.short_path_dict_2[self.end_hub][0]
                            print("new_cost:", new_cost)
                            drone.start_turn = turn + added_cost
                            print("hi2")
                            if new_cost <= self.cost + added_cost:
                                # turn = 0
                                drone.start_turn = turn
                                print("hi3")
                                for zone in self.short_path2:
                                    # print(turn)
                                    if len(turnes) < turn:
                                        # print("kk")
                                        turnes.append((turn, [n['name'] for n in self.hubs.values() if n['name'] not in [self.start_hub]]))
                                    if len(turnes) >= turn:
                                        # print("jj")
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
                                                    if len(turnes) < turn:
                                                        turnes.append((turn, [n['name'] for n in self.hubs.values() if n['name'] not in [self.start_hub]]))
                                                turnes[turn - 1][1].remove(zone)
                                            # break
                                    turn += 1
                                    # else:
                                    #     turnes.append((turn, [n['name'] for n in self.hubs.values() if n['name'] not in [self.start_hub]]))
                                    #     drone.path.append(zone)
                                    #     print(zone)

                                    #     turnes[turn - 1][1].remove(zone)
                                break
                    
                        else:
                            pass
                            looking_for_cost = math.inf
                            looking_for_path = []

                            for path in self.all_paths:
                                if path[0] < looking_for_cost:
                                    looking_for_cost = path[0]
                                    looking_for_path = path

                            new_zone = looking_for_path[1][0]
                            added_new_cost = 0
                            for t in turnes:
                                if new_zone not in t[1]:
                                    added_new_cost += 1
                                else:
                                    break
                            drone.start_turn = turn + added_new_cost
                            print("hi4")
                            print("added_new_cost", added_new_cost)
                            
                            drone.path = looking_for_path[1]
                            turn = drone.start_turn
                            print("drone.start_turn", drone.start_turn)
                            # for path in drone.path:
                            #     print("a")
                            #     for k, v in self.hubs.items():
                            #         if v['name'] == path:
                            #             zone_name = v['metadata']['zone']
                            #     # print(zone_name)
                            #     if zone_name == "restricted":
                            #         stay = 2
                            #     elif zone_name == "normal":
                            #         stay = 1
                            #     # print(turnes)
                            #     for i in range(stay):
                            #         print(turn)
                            #         print(path)
                            #         # print(turnes)
                            #         if i >= 1:
                            #             turn += 1
                            #             if len(turnes) < turn:
                            #                 turnes.append((turn, [n['name'] for n in self.hubs.values() if n['name'] not in [self.start_hub]]))
                            #         turnes[turn - 1][1].remove(path)
                            #         print(turnes)
                            #     turn += 1
                            for path in drone.path:
                                added_turns = 0
                                if len(turnes) < turn:
                                    turnes.append((turn, [n['name'] for n in self.hubs.values() if n['name'] not in [self.start_hub]]))
                                if path in turnes[turn - 1][1]:
                                    turnes[turn - 1][1].remove(path)
                                    print("---------remove(path)1:", path, "from", turn)
                                else:
                                    while True:
                                        i = drone.path.index(path)
                                        drone.path.insert(i, drone.path[i - 1])
                                        print("---------drone.path[i - 1]", drone.path[i - 1], "from", turn)
                                        turnes[turn - 1][1].remove(drone.path[i - 1])
                                        turn += 1
                                        added_turns += 1
                                        if path in turnes[turn - 1][1]:
                                            break
                                # print(turnes)
                                turn -= added_turns
                                turn += 1
                            looking_for_path[0] = added_new_cost + len(drone.path) + 1
                            break

                else:
                    turnes.append((turn, [n['name'] for n in self.hubs.values() if n['name'] not in  [self.start_hub]]))
                    if zone in turnes[turn - 1][1]:
                        drone.start_turn = turn
                        print("hi1")
                        for k, v in self.hubs.items():
                            if v['name'] == zone:
                                print("zone", zone)
                                zone_name = v['metadata']['zone']
                        if zone_name == "restricted":
                            stay = 2
                        else:
                            stay = 1
                        for i in range(stay):
                            drone.path.append(zone)
                            if i >= 1:
                                turn += 1
                                turnes.append((turn, [n['name'] for n in self.hubs.values() if n['name'] not in  [self.start_hub]]))
                            turnes[turn - 1][1].remove(zone)
                        print("drone.path gggggg", drone.path)


                    else:
                        is_found = 0
                        for e in self.neighbours[self.short_path_dict[zone][1]]:
                            for k, v in self.hubs.items():
                                if v['name'] == zone:
                                    zone_name = v['metadata']['zone']
                            if e in turnes[turn - 1][1] and zone_name != "blocked":
                                is_found = 1
                                break
                        if is_found:
                            print("else")
                            added_cost = 0
                            for t in turnes:
                                if zone not in t[1]:
                                    added_cost += 1
                                else:
                                    break
                            print("cost", self.cost)
                            print("added_cost", added_cost)

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

                                heap = []

                                for alt_zone, (cost, from_hub2) in self.short_path_dict_2.items():
                                    if alt_zone in self.unvisited2:
                                        heapq.heappush(heap, (cost, from_hub2, alt_zone))


                                smallest_cost, from_hub2, lower_cost_zone_name = heapq.heappop(heap)
                                # if from_hub2 != self.start_hub:
                                self.unvisited2.remove(lower_cost_zone_name)

                                from_hub2 = lower_cost_zone_name
                                
                                if not lower_cost_zone_name or lower_cost_zone_name == self.end_hub:
                                    break
                                else:
                                    initiate_neighbours = list(self.neighbours[lower_cost_zone_name])
                            self.short_path2 = []
                            a = self.end_hub
                            self.short_path2.append(a)
                            while a != sstart_hub:
                                self.short_path2.append(self.short_path_dict_2[a][1])
                                a = self.short_path_dict_2[a][1]
                            self.short_path2.reverse()
                            self.short_path2.remove(sstart_hub)
                            self.short_path2.remove(self.end_hub)
                            new_cost = self.short_path_dict_2[self.end_hub][0]
                            print("new_cost:", new_cost)
                            drone.start_turn = turn + added_cost
                            print("hi2")
                            if new_cost <= self.cost + added_cost:
                                # turn = 0
                                drone.start_turn = turn
                                print("hi3")
                                for zone in self.short_path2:
                                    # print(turn)
                                    if len(turnes) < turn:
                                        # print("kk")
                                        turnes.append((turn, [n['name'] for n in self.hubs.values() if n['name'] not in [self.start_hub]]))
                                    if len(turnes) >= turn:
                                        # print("jj")
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
                                                    if len(turnes) < turn:
                                                        turnes.append((turn, [n['name'] for n in self.hubs.values() if n['name'] not in [self.start_hub]]))
                                                turnes[turn - 1][1].remove(zone)
                                            # break
                                    turn += 1
                                    # else:
                                    #     turnes.append((turn, [n['name'] for n in self.hubs.values() if n['name'] not in [self.start_hub]]))
                                    #     drone.path.append(zone)
                                    #     print(zone)

                                    #     turnes[turn - 1][1].remove(zone)
                                break
                    
                        else:
                            pass
                            looking_for_cost = math.inf
                            looking_for_path = []

                            for path in self.all_paths:
                                if path[0] < looking_for_cost:
                                    looking_for_cost = path[0]
                                    looking_for_path = path

                            new_zone = looking_for_path[1][0]
                            added_new_cost = 0
                            for t in turnes:
                                if new_zone not in t[1]:
                                    added_new_cost += 1
                                else:
                                    break
                            drone.start_turn = turn + added_new_cost
                            print("hi4")
                            print("added_new_cost", added_new_cost)
                            
                            drone.path = looking_for_path[1]
                            turn = drone.start_turn
                            print("drone.start_turn", drone.start_turn)
                            # for path in drone.path:
                            #     print("a")
                            #     for k, v in self.hubs.items():
                            #         if v['name'] == path:
                            #             zone_name = v['metadata']['zone']
                            #     # print(zone_name)
                            #     if zone_name == "restricted":
                            #         stay = 2
                            #     elif zone_name == "normal":
                            #         stay = 1
                            #     # print(turnes)
                            #     for i in range(stay):
                            #         print(turn)
                            #         print(path)
                            #         # print(turnes)
                            #         if i >= 1:
                            #             turn += 1
                            #             if len(turnes) < turn:
                            #                 turnes.append((turn, [n['name'] for n in self.hubs.values() if n['name'] not in [self.start_hub]]))
                            #         turnes[turn - 1][1].remove(path)
                            #         print(turnes)
                            #     turn += 1
                            for path in drone.path:
                                added_turns = 0
                                if len(turnes) < turn:
                                    turnes.append((turn, [n['name'] for n in self.hubs.values() if n['name'] not in [self.start_hub]]))
                                if path in turnes[turn - 1][1]:
                                    turnes[turn - 1][1].remove(path)
                                    print("---------remove(path)1:", path, "from", turn)
                                else:
                                    while True:
                                        i = drone.path.index(path)
                                        drone.path.insert(i, drone.path[i - 1])
                                        print("---------drone.path[i - 1]", drone.path[i - 1], "from", turn)
                                        turnes[turn - 1][1].remove(drone.path[i - 1])
                                        turn += 1
                                        added_turns += 1
                                        if path in turnes[turn - 1][1]:
                                            break
                                # print(turnes)
                                turn -= added_turns
                                turn += 1
                            looking_for_path[0] = added_new_cost + len(drone.path) + 1
                            break































                # print(turnes)
            can_i_add = 1
            for e in self.all_paths:
                if drone.path == e[1]:
                    can_i_add = 0
                    break
            if can_i_add:
                self.all_paths.append([len(drone.path) + 1, drone.path])
            print("turnes:", turnes)
            print("drone.path:", drone.path)
            print("drone.start_turn", drone.start_turn)
            print("self.all_paths:", self.all_paths)
            print("-------------------------------------")
            print(turnes[-1][0] + 1)
