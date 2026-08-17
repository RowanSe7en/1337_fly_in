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

    def _shortest_path_search(self, start_hub, end_hub, unvisited_list=None,
                               exclude_zones=None, allowed_zones=None,
                               skip_blocked=True, from_placeholder="none"):

        if unvisited_list is None:
            unvisited_list = [v['name'] for v in self.hubs.values()]

        path_dict = {}
        for e in unvisited_list:
            path_dict[e] = (math.inf, "none")

        from_hub = start_hub
        path_dict[from_hub] = (0, from_placeholder)

        initiate_neighbours = list(self.neighbours[from_hub])
        cost = 0
        unvisited_list.remove(from_hub)
        i = 0

        while initiate_neighbours:
            i += 1

            for zone in initiate_neighbours:
                is_blocked = 0
                if zone in unvisited_list and (exclude_zones is None or zone not in exclude_zones):
                    for hub_data in self.hubs.values():
                        if hub_data['name'] == zone:
                            if hub_data['metadata']['zone'] == "blocked":
                                if skip_blocked:
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

                    if allowed_zones is None:
                        if zone in path_dict:
                            if path_dict[zone][0] > path_dict[from_hub][0] + cost:
                                path_dict[zone] = (path_dict[from_hub][0] + cost, from_hub)
                        else:
                            path_dict[zone] = (path_dict[from_hub][0] + cost, from_hub)
                    else:
                        if zone in path_dict and zone in allowed_zones:
                            if path_dict[zone][0] > path_dict[from_hub][0] + cost:
                                path_dict[zone] = (path_dict[from_hub][0] + cost, from_hub)
                        else:
                            if zone in allowed_zones:
                                path_dict[zone] = (path_dict[from_hub][0] + cost, from_hub)

            heap = []
            for zone, (zone_cost, zone_from_hub) in path_dict.items():
                if zone in unvisited_list:
                    heapq.heappush(heap, (zone_cost, zone_from_hub, zone))

            smallest_cost, from_hub, lower_cost_zone_name = heapq.heappop(heap)
            unvisited_list.remove(lower_cost_zone_name)
            from_hub = lower_cost_zone_name

            if not lower_cost_zone_name or lower_cost_zone_name == end_hub:
                break
            else:
                initiate_neighbours = list(self.neighbours[lower_cost_zone_name])

        path = []
        a = end_hub
        path.append(a)
        while a != start_hub:
            path.append(path_dict[a][1])
            a = path_dict[a][1]
        path.reverse()
        path.remove(start_hub)
        path.remove(end_hub)

        return path, path_dict[end_hub][0], path_dict, unvisited_list

    def find_the_shortest_path(self):

        self.short_path, self.cost, self.short_path_dict, self.unvisited = self._shortest_path_search(
            start_hub=self.start_hub,
            end_hub=self.end_hub,
            unvisited_list=self.unvisited,
        )
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
                    if zone in turnes[turn - 1][1]:
                        drone.start_turn = turn
                        for k, v in self.hubs.items():
                            if v['name'] == zone:
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
                    else:
                        is_found = 0
                        for e in self.neighbours[self.short_path_dict[zone][1]]:
                            for k, v in self.hubs.items():
                                if v['name'] == e:
                                    zone_name = v['metadata']['zone']
                            if e in turnes[turn - 1][1] and zone_name != "blocked":
                                is_found = 1
                                break
                        if is_found:
                            added_cost = 0
                            for t in turnes:
                                if zone not in t[1]:
                                    added_cost += 1
                                else:
                                    break

                            from_hub2 = self.short_path_dict[zone][1]  # this is correct
                            sstart_hub = from_hub2

                            self.short_path2, new_cost, self.short_path_dict_2, self.unvisited2 = self._shortest_path_search(
                                start_hub=from_hub2,
                                end_hub=self.end_hub,
                                exclude_zones=[zone, self.short_path_dict[zone][1]],
                                allowed_zones=turnes[turn - 1][1],
                                skip_blocked=False,
                                from_placeholder=self.short_path_dict[from_hub2][1],
                            )
                            drone.start_turn = turn + added_cost
                            if new_cost <= self.cost + added_cost:
                                # turn = 0
                                drone.start_turn = turn
                                for zone in self.short_path2:
                                    if len(turnes) < turn:
                                        turnes.append((turn, [n['name'] for n in self.hubs.values() if n['name'] not in [self.start_hub]]))
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
                                                    if len(turnes) < turn:
                                                        turnes.append((turn, [n['name'] for n in self.hubs.values() if n['name'] not in [self.start_hub]]))
                                                turnes[turn - 1][1].remove(zone)
                                            # break
                                    turn += 1

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
                            
                            drone.path = looking_for_path[1]
                            turn = drone.start_turn
                            print("drone.start_turn", drone.start_turn)

                            for path in drone.path:
                                added_turns = 0
                                if len(turnes) < turn:
                                    turnes.append((turn, [n['name'] for n in self.hubs.values() if n['name'] not in [self.start_hub]]))
                                if path in turnes[turn - 1][1]:
                                    turnes[turn - 1][1].remove(path)
                                else:
                                    while True:
                                        i = drone.path.index(path)
                                        drone.path.insert(i, drone.path[i - 1])
                                        turnes[turn - 1][1].remove(drone.path[i - 1])
                                        turn += 1
                                        added_turns += 1
                                        if path in turnes[turn - 1][1]:
                                            break
                                turn -= added_turns
                                turn += 1
                            looking_for_path[0] = added_new_cost + len(drone.path) + 1
                            break

                else:
                    turnes.append((turn, [n['name'] for n in self.hubs.values() if n['name'] not in  [self.start_hub]]))
                    if zone in turnes[turn - 1][1]:
                        drone.start_turn = turn
                        for k, v in self.hubs.items():
                            if v['name'] == zone:
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


                    else:
                        is_found = 0
                        for e in self.neighbours[self.short_path_dict[zone][1]]:
                            for k, v in self.hubs.items():
                                if v['name'] == e:
                                    zone_name = v['metadata']['zone']
                            if e in turnes[turn - 1][1] and zone_name != "blocked":
                                is_found = 1
                                break
                        if is_found:
                            added_cost = 0
                            for t in turnes:
                                if zone not in t[1]:
                                    added_cost += 1
                                else:
                                    break

                            from_hub2 = self.short_path_dict[zone][1]  # this is correct
                            sstart_hub = from_hub2

                            self.short_path2, new_cost, self.short_path_dict_2, self.unvisited2 = self._shortest_path_search(
                                start_hub=from_hub2,
                                end_hub=self.end_hub,
                                exclude_zones=[zone, self.short_path_dict[zone][1]],
                                allowed_zones=turnes[turn - 1][1],
                                skip_blocked=False,
                                from_placeholder=self.short_path_dict[from_hub2][1],
                            )
                            drone.start_turn = turn + added_cost
                            if new_cost <= self.cost + added_cost:
                                # turn = 0
                                drone.start_turn = turn
                                for zone in self.short_path2:
                                    if len(turnes) < turn:
                                        turnes.append((turn, [n['name'] for n in self.hubs.values() if n['name'] not in [self.start_hub]]))
                                    if len(turnes) >= turn:
                                        if zone in turnes[turn - 1][1]:
                                            for k, v in self.hubs.items():
                                                if v['name'] == zone:
                                                    zone_name = v['metadata']['zone']
                                                    break
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
                            
                            drone.path = looking_for_path[1]
                            turn = drone.start_turn
                            print("drone.start_turn", drone.start_turn)

                            for path in drone.path:
                                added_turns = 0
                                if len(turnes) < turn:
                                    turnes.append((turn, [n['name'] for n in self.hubs.values() if n['name'] not in [self.start_hub]]))
                                if path in turnes[turn - 1][1]:
                                    turnes[turn - 1][1].remove(path)
                                else:
                                    while True:
                                        i = drone.path.index(path)
                                        drone.path.insert(i, drone.path[i - 1])
                                        turnes[turn - 1][1].remove(drone.path[i - 1])
                                        turn += 1
                                        added_turns += 1
                                        if path in turnes[turn - 1][1]:
                                            break
                                turn -= added_turns
                                turn += 1
                            looking_for_path[0] = added_new_cost + len(drone.path) + 1
                            break

            can_i_add = 1
            for e in self.all_paths:
                if drone.path == e[1]:
                    can_i_add = 0
                    break
            if can_i_add:
                self.all_paths.append([len(drone.path) + 1, drone.path])
            print("turnes:", turnes)
            print("drone.path:", drone.path)
            print("self.all_paths:", self.all_paths)
            print("-------------------------------------")
            print(turnes[-1][0] + 1)
