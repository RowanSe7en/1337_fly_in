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

        return path, path_dict[end_hub][0], path_dict, unvisited_list

    def find_the_shortest_path(self):

        self.short_path, self.cost, self.short_path_dict, self.unvisited = self._shortest_path_search(
            start_hub=self.start_hub,
            end_hub=self.end_hub,
            unvisited_list=self.unvisited,
        )
        print(self.short_path)

    def _get_zone_name(self, zone):
        for k, v in self.hubs.items():
            if v['name'] == zone:
                return v['metadata']['zone']

        return None

    def _normalize_zone_turns(self, zones_at_turnes):
        """Return snapshots numbered consecutively starting at turn 0."""
        if not zones_at_turnes:
            return []

        normalized = []
        previous = None

        for _old_turn, zones in zones_at_turnes:
            snapshot = {
                zone: list(drone_ids)
                for zone, drone_ids in zones.items()
            }
            normalized.append(snapshot)

        return [
            (index, zones)
            for index, zones in enumerate(normalized)
        ]

    def _ensure_zone_turn(self, zones_at_turnes, turn):
        """Ensure a zone-state snapshot exists for a turn."""
        while len(zones_at_turnes) <= turn:
            previous = zones_at_turnes[-1][1]
            next_turn = len(zones_at_turnes)
            zones_at_turnes.append((
                next_turn,
                {zone: list(drone_ids) for zone, drone_ids in previous.items()}
            ))

    def _move_drone(self, drone, zone, turn, zones_at_turnes):
        """Record the drone's position for the given turn."""
        self._ensure_zone_turn(zones_at_turnes, turn)
        zones = zones_at_turnes[turn][1]

        for drone_ids in zones.values():
            if drone.id in drone_ids:
                drone_ids.remove(drone.id)

        if drone.id not in zones[zone]:
            zones[zone].append(drone.id)

    def _zone_has_capacity(self, zone, turn, zones_at_turnes):
        """Return True if the zone can accept another drone."""

        if zone == self.end_hub:
            return True

        current_drones = len(
            zones_at_turnes[turn][1][zone]
        )

        for hub in self.hubs.values():
            if hub["name"] == zone:
                max_drones = hub["metadata"]["max_drones"]

                return current_drones + 1 < max_drones

        return False

    def _process_zone(self, drone, zone, turn, turnes, zones_at_turnes):

        if zone in turnes[turn - 1][1]:

            drone.start_turn = turn

            zone_name = self._get_zone_name(zone)

            if zone_name == "restricted":
                stay = 2
            else:
                stay = 1

            for i in range(stay):
                drone.path.append(zone)

                if i >= 1:
                    turn += 1
                    turnes.append((
                        turn,
                        [
                            n['name'] for n in self.hubs.values()
                            if n['name'] not in [self.start_hub]
                        ]
                    ))
                    self._ensure_zone_turn(zones_at_turnes, turn)
                self._ensure_zone_turn(zones_at_turnes, turn)

                if zone != self.end_hub and not self._zone_has_capacity(zone, turn, zones_at_turnes):
                    turnes[turn - 1][1].remove(zone)
                self._move_drone(drone, zone, turn, zones_at_turnes)

            return turn, False

        else:
            is_found = 0

            for e in self.neighbours[self.short_path_dict[zone][1]]:
                zone_name = self._get_zone_name(zone)

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

                from_hub = self.short_path_dict[zone][1]
                start_hub = from_hub

                self.short_path, new_cost, self.short_path_dict, self.unvisited = (
                    self._shortest_path_search(
                        start_hub=from_hub,
                        end_hub=self.end_hub,
                        exclude_zones=[
                            zone,
                            self.short_path_dict[zone][1]
                        ],
                        allowed_zones=turnes[turn - 1][1],
                        skip_blocked=False,
                        from_placeholder=self.short_path_dict[from_hub][1],
                    )
                )

                drone.start_turn = turn + added_cost

                if new_cost <= self.cost + added_cost:
                    drone.start_turn = turn

                    for zone in self.short_path:
                        if len(turnes) < turn:
                            turnes.append((
                                turn,
                                [
                                    n['name'] for n in self.hubs.values()
                                    if n['name'] not in [self.start_hub]
                                ]
                            ))

                        if len(turnes) >= turn:
                            if zone in turnes[turn - 1][1]:
                                zone_name = self._get_zone_name(zone)

                                if zone_name == "restricted":
                                    stay = 2
                                elif zone_name == "normal":
                                    stay = 1

                                for i in range(stay):
                                    drone.path.append(zone)

                                    if i >= 1:
                                        turn += 1

                                        if len(turnes) < turn:
                                            turnes.append((
                                                turn,
                                                [
                                                    n['name']
                                                    for n in self.hubs.values()
                                                    if n['name']
                                                    not in [self.start_hub]
                                                ]
                                            ))

                                    if zone != self.end_hub and not self._zone_has_capacity(zone, turn, zones_at_turnes):
                                        turnes[turn - 1][1].remove(zone)
                                    self._move_drone(drone, zone, turn, zones_at_turnes)

                        turn += 1

                    return turn, True

            else:
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
                        turnes.append((
                            turn,
                            [
                                n['name'] for n in self.hubs.values()
                                if n['name'] not in [self.start_hub]
                            ]
                        ))
                    self._ensure_zone_turn(zones_at_turnes, turn)

                    if path in turnes[turn - 1][1]:

                        if path != self.end_hub and not self._zone_has_capacity(zone, turn, zones_at_turnes):
                            turnes[turn - 1][1].remove(path)
                        self._move_drone(drone, path, turn, zones_at_turnes)

                    else:
                        while True:
                            i = drone.path.index(path)
                            drone.path.insert(i, drone.path[i - 1])

                            if drone.path[i - 1] != self.end_hub and not self._zone_has_capacity(zone, turn, zones_at_turnes):
                                turnes[turn - 1][1].remove(drone.path[i - 1])
                            self._move_drone(
                                drone, drone.path[i - 1], turn, zones_at_turnes
                            )
                            turn += 1
                            added_turns += 1

                            if path in turnes[turn - 1][1]:
                                break

                    turn -= added_turns
                    turn += 1

                looking_for_path[0] = (
                    added_new_cost + len(drone.path) + 1
                )

                return turn, True

        return turn, False

    def ecah_drone_path_assigner(self):

        turnes = [(1, [n['name'] for n in self.hubs.values() if n['name'] not in [self.start_hub]])]
        zones_at_turnes = [
            (
                0,
                {
                    hub_data["name"]: (
                        [drone.id for drone in drones_list]
                        if hub_data["name"] == self.start_hub
                        else []
                    )
                    for hub_data in self.hubs.values()
                }
            )
        ]
        zones_at_turnes[:] = self._normalize_zone_turns(zones_at_turnes)

        original_short_path = list(self.short_path)
        original_short_path_dict = dict(self.short_path_dict)
        original_unvisited = list(self.unvisited)

        for drone in drones_list:
            self.short_path = list(original_short_path)
            self.short_path_dict = dict(original_short_path_dict)
            self.unvisited = list(original_unvisited)

            print("drone.id", drone.id)
            turn = 0
            cost = 0
            for zone in self.short_path:
                turn += 1

                if len(turnes) >= turn:
                    turn, should_break = self._process_zone(
                        drone,
                        zone,
                        turn,
                        turnes,
                        zones_at_turnes
                    )

                    if should_break:
                        break

                else:
                    turnes.append((
                        turn,
                        [
                            n['name'] for n in self.hubs.values()
                            if n['name'] not in [self.start_hub]
                        ]
                    ))
                    self._ensure_zone_turn(zones_at_turnes, turn)

                    turn, should_break = self._process_zone(
                        drone,
                        zone,
                        turn,
                        turnes,
                        zones_at_turnes
                    )

                    if should_break:
                        break

            can_i_add = 1
            for e in self.all_paths:
                if drone.path == e[1]:
                    can_i_add = 0
                    break
            if can_i_add:
                self.all_paths.append([len(drone.path) + 1, drone.path])
            print("turnes:", turnes)
            zones_at_turnes[:] = self._normalize_zone_turns(zones_at_turnes)
            print("zones_at_turnes:", zones_at_turnes)
            print("drone.path:", drone.path)
            print("self.all_paths:", self.all_paths)
            print("-------------------------------------")
            print(turnes[-1][0] + 1)
        return zones_at_turnes
