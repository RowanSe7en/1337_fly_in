import math
import heapq
from drones import drones_list
import copy


class Algo:

    def __init__(self, data):
        self.data = data

        self.hubs, self.connections, self.nb_drones, self.neighbours = self.data

        self.start_hub = self.hubs['start_hub']['name']
        self.end_hub = self.hubs['end_hub']['name']

        self.unvisited = [v['name'] for v in self.hubs.values()]

        self.cost = 0
        self.all_paths = []

        self.turnes = []
        self.zones_at_turnes = []

    def shortest_path_search(self, start_hub, end_hub, unvisited_list=None,
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

        if not math.isfinite(path_dict[end_hub][0]):
            return [], path_dict[end_hub][0], path_dict, unvisited_list

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

        self.short_path, self.cost, self.short_path_dict, self.unvisited = self.shortest_path_search(
            start_hub=self.start_hub,
            end_hub=self.end_hub,
            unvisited_list=self.unvisited,
        )

    def get_zone_name(self, zone):
        for k, v in self.hubs.items():
            if v['name'] == zone:
                return v['metadata']['zone']

        return None

    def normalize_zone_turns(self):

        if not self.zones_at_turnes:
            return []

        normalized = []
        previous = None

        for _old_turn, zones in self.zones_at_turnes:
            snapshot = {
                zone: list(drone_ids)
                for zone, drone_ids in zones.items()
            }
            normalized.append(snapshot)

        return [
            (index, zones)
            for index, zones in enumerate(normalized)
        ]

    def ensure_zone_turn(self, turn):

        while len(self.zones_at_turnes) <= turn:
            previous = self.zones_at_turnes[-1][1]
            next_turn = len(self.zones_at_turnes)
            self.zones_at_turnes.append((
                next_turn,
                {zone: list(drone_ids) for zone, drone_ids in previous.items()}
            ))

    def drone_at_which_zone(self, drone, snapshot):

        for zone_name, drone_ids in snapshot.items():
            if drone.id in drone_ids:
                return zone_name
        return None

    def update_future_zone_turns(self, drone, changed_turn, old_zone, new_zone):

        if old_zone == new_zone or old_zone is None:
            return

        for t in range(changed_turn + 1, len(self.zones_at_turnes)):
            snapshot = self.zones_at_turnes[t][1]

            if self.drone_at_which_zone(drone, snapshot) != old_zone:
                break

            for drone_ids in snapshot.values():
                if drone.id in drone_ids:
                    drone_ids.remove(drone.id)
            if drone.id not in snapshot[new_zone]:
                snapshot[new_zone].append(drone.id)

    def move_drone(self, drone, zone, turn, ff):

        self.ensure_zone_turn(turn)
        zones = self.zones_at_turnes[turn][1]

        old_zone = self.drone_at_which_zone(drone, zones)

        for v in self.hubs.values():
            if v["name"] == old_zone:
                max_drones = v["metadata"]["max_drones"]

                if len(self.zones_at_turnes[turn][1][old_zone]) < max_drones:
                    if old_zone not in self.turnes[turn - 1][1]:
                        self.turnes[turn - 1][1].append(old_zone)

                break

        for drone_ids in zones.values():
            if drone.id in drone_ids:
                drone_ids.remove(drone.id)

        if drone.id not in zones[zone]:
            zones[zone].append(drone.id)

        self.update_future_zone_turns(drone, turn, old_zone, zone)

    def zone_has_capacity(self, zone, turn):

        if zone == self.end_hub:
            return True

        self.ensure_zone_turn(turn)
        current_drones = len(
            self.zones_at_turnes[turn][1][zone]
        )

        for hub in self.hubs.values():
            if hub["name"] == zone:
                max_drones = hub["metadata"]["max_drones"]

                return current_drones + 1 < max_drones

        return False

    def process_zone(self, drone, zone, turn):

        if zone in self.turnes[turn - 1][1]:

            drone.start_turn = turn

            zone_name = self.get_zone_name(zone)

            if zone_name == "restricted":
                stay = 2
            else:
                stay = 1

            for i in range(stay):
                drone.path.append(zone)

                if i >= 1:
                    turn += 1
                    self.turnes.append((
                        turn,
                        [
                            n['name'] for n in self.hubs.values()
                            if n['name'] not in [self.start_hub]
                        ]
                    ))
                    self.ensure_zone_turn(turn)
                self.ensure_zone_turn(turn)

                if zone != self.end_hub and not self.zone_has_capacity(zone, turn):
                    self.turnes[turn - 1][1].remove(zone)
                self.move_drone(drone, zone, turn, "kayn")
            return turn, False

        else:
            is_found = 0

            for e in self.neighbours[self.short_path_dict[zone][1]]:
                zone_name = self.get_zone_name(zone)

                if e in self.turnes[turn - 1][1] and zone_name != "blocked":
                    is_found = 1
                    break
            if is_found:

                added_cost = 0

                for t in self.turnes:
                    if zone not in t[1]:
                        added_cost += 1
                    else:
                        break

                from_hub = self.short_path_dict[zone][1]
                start_hub = from_hub

                saved_short_path = list(self.short_path)
                saved_short_path_dict = dict(self.short_path_dict)
                saved_unvisited = list(self.unvisited)
                

                self.short_path, new_cost, self.short_path_dict, self.unvisited = (
                    self.shortest_path_search(
                        start_hub=from_hub,
                        end_hub=self.end_hub,
                        exclude_zones=[
                            zone,
                            self.short_path_dict[zone][1],
                            self.short_path_dict[from_hub][1]
                        ],
                        allowed_zones=self.turnes[turn - 1][1],
                        skip_blocked=False,
                        from_placeholder=self.short_path_dict[from_hub][1],
                    )
                )

                drone.start_turn = turn + added_cost

                if math.isfinite(new_cost) and new_cost <= self.cost + added_cost:

                    drone.start_turn = turn

                    for zone in self.short_path:
                        if len(self.turnes) < turn:
                            self.turnes.append((
                                turn,
                                [
                                    n['name'] for n in self.hubs.values()
                                    if n['name'] not in [self.start_hub]
                                ]
                            ))

                        if len(self.turnes) >= turn:
                            if zone in self.turnes[turn - 1][1]:
                                zone_name = self.get_zone_name(zone)

                                if zone_name == "restricted":
                                    stay = 2
                                elif zone_name == "normal":
                                    stay = 1

                                for i in range(stay):
                                    drone.path.append(zone)

                                    if i >= 1:
                                        turn += 1

                                        if len(self.turnes) < turn:
                                            self.turnes.append((
                                                turn,
                                                [
                                                    n['name']
                                                    for n in self.hubs.values()
                                                    if n['name']
                                                    not in [self.start_hub]
                                                ]
                                            ))

                                    if zone != self.end_hub and not self.zone_has_capacity(zone, turn):
                                        self.turnes[turn - 1][1].remove(zone)
                                    self.move_drone(drone, zone, turn, "badil")

                        turn += 1

                    return turn, True

                self.short_path = saved_short_path
                self.short_path_dict = saved_short_path_dict
                self.unvisited = saved_unvisited
                self.zones_at_turnes = self.saved_zones_at_turnes
                self.turnes = self.saved_turnes
                is_found = 0

            if not is_found:

                self.turnes = self.saved_turnes
                self.zones_at_turnes = self.saved_zones_at_turnes
                turn = 1


                looking_for_cost = math.inf
                looking_for_path = []

                for path in self.all_paths:
                    if path[0] < looking_for_cost:
                        looking_for_cost = path[0]
                        looking_for_path = path

                new_zone = looking_for_path[1][0]
                added_new_cost = 0

                for t in self.turnes:
                    if new_zone not in t[1]:
                        added_new_cost += 1
                    else:
                        break

                drone.start_turn = turn + added_new_cost

                drone.path = looking_for_path[1]
                turn = drone.start_turn


                for path in drone.path:
                    added_turns = 0

                    if len(self.turnes) < turn:
                        self.turnes.append((
                            turn,
                            [
                                n['name'] for n in self.hubs.values()
                                if n['name'] not in [self.start_hub]
                            ]
                        ))
                    self.ensure_zone_turn(turn)

                    if path in self.turnes[turn - 1][1]:

                        if path != self.end_hub and not self.zone_has_capacity(path, turn):
                            self.turnes[turn - 1][1].remove(path)
                        self.move_drone(drone, path, turn, "no")

                    else:
                        while True:
                            i = drone.path.index(path)
                            drone.path.insert(i, drone.path[i - 1])

                            if drone.path[i - 1] != self.end_hub and not self.zone_has_capacity(drone.path[i - 1], turn):
                                self.turnes[turn - 1][1].remove(drone.path[i - 1])
                            self.move_drone(
                                drone, drone.path[i - 1], turn, "no2"
                            )
                            turn += 1
                            added_turns += 1

                            if len(self.turnes) < turn:
                                self.turnes.append((
                                    turn,
                                    [
                                        n['name'] for n in self.hubs.values()
                                        if n['name'] not in [self.start_hub]
                                    ]
                                ))

                            if path in self.turnes[turn - 1][1]:
                                break

                    turn -= added_turns
                    turn += 1

                looking_for_path[0] = (
                    added_new_cost + len(drone.path) + 1
                )

                return turn, True

        return turn, False

    def ecah_drone_path_assigner(self):

        self.turnes = [(1, [n['name'] for n in self.hubs.values() if n['name'] not in [self.start_hub]])]
        self.zones_at_turnes = [
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
        self.zones_at_turnes[:] = self.normalize_zone_turns()

        original_short_path = list(self.short_path)
        original_short_path_dict = dict(self.short_path_dict)
        original_unvisited = list(self.unvisited)

        for drone in drones_list:

            self.short_path = list(original_short_path)
            self.short_path_dict = dict(original_short_path_dict)
            self.unvisited = list(original_unvisited)

            self.saved_turnes = copy.deepcopy(self.turnes)
            self.saved_zones_at_turnes = copy.deepcopy(self.zones_at_turnes)

            # print("drone.id", drone.id)
            turn = 0
            cost = 0
            for zone in self.short_path:
                turn += 1

                if len(self.turnes) >= turn:
                    turn, should_break = self.process_zone(
                        drone,
                        zone,
                        turn,
                    )

                    if should_break:
                        break

                else:
                    self.turnes.append((
                        turn,
                        [
                            n['name'] for n in self.hubs.values()
                            if n['name'] not in [self.start_hub]
                        ]
                    ))
                    self.ensure_zone_turn(turn)

                    turn, should_break = self.process_zone(
                        drone,
                        zone,
                        turn,
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
            # print("turnes:", self.turnes)
            self.zones_at_turnes[:] = self.normalize_zone_turns()
            # print("zones_at_turnes:", self.zones_at_turnes)
            # print("drone.path:", drone.path)
            # print("self.all_paths:", self.all_paths)
            # print("-------------------------------------")
            # print(self.turnes[-1][0] + 1)
        return self.zones_at_turnes

    def _connection_name(self, from_zone, to_zone):

        candidate_keys = [
            (from_zone, to_zone),
            (to_zone, from_zone),
            f"{from_zone}-{to_zone}",
            f"{to_zone}-{from_zone}",
            frozenset((from_zone, to_zone)),
        ]

        connections = self.connections
        if isinstance(connections, dict):
            for key in candidate_keys:
                try:
                    if key in connections:
                        value = connections[key]
                        if isinstance(value, dict):
                            return value.get(
                                'name',
                                key if isinstance(key, str) else f"{from_zone}-{to_zone}"
                            )
                        if isinstance(value, str):
                            return value
                        return key if isinstance(key, str) else f"{from_zone}-{to_zone}"
                except TypeError:
                    continue

        elif isinstance(connections, (list, tuple, set)):
            for conn in connections:
                if isinstance(conn, dict):
                    a = conn.get('from') or conn.get('a') or conn.get('start') or conn.get('start_hub')
                    b = conn.get('to') or conn.get('b') or conn.get('end') or conn.get('end_hub')
                    if a is not None and b is not None and {a, b} == {from_zone, to_zone}:
                        return conn.get('name', f"{from_zone}-{to_zone}")
                elif isinstance(conn, str) and from_zone in conn and to_zone in conn:
                    return conn

        return f"{from_zone}-{to_zone}"

    def print_simulation(self):

        if not self.zones_at_turnes:
            return

        total_drones = len(drones_list)
        delivered = set()

        pending_restricted_arrival = {}

        for t in range(1, len(self.zones_at_turnes)):
            if len(delivered) >= total_drones:
                break

            prev_snapshot = self.zones_at_turnes[t - 1][1]
            curr_snapshot = self.zones_at_turnes[t][1]

            moves = []

            for drone in drones_list:
                if drone.id in delivered:
                    continue

                prev_zone = self.drone_at_which_zone(drone, prev_snapshot)
                curr_zone = self.drone_at_which_zone(drone, curr_snapshot)

                if curr_zone is None:
                    continue

                if curr_zone == prev_zone:
                    if pending_restricted_arrival.get(drone.id) == curr_zone:
                        moves.append((drone.id, f"D{drone.id}-{curr_zone}"))
                        del pending_restricted_arrival[drone.id]
                        if curr_zone == self.end_hub:
                            delivered.add(drone.id)
                    continue

                if self.get_zone_name(curr_zone) == "restricted":
                    connection = self._connection_name(prev_zone, curr_zone)
                    moves.append((drone.id, f"D{drone.id}-{connection}"))
                    pending_restricted_arrival[drone.id] = curr_zone
                else:
                    moves.append((drone.id, f"D{drone.id}-{curr_zone}"))
                    if curr_zone == self.end_hub:
                        delivered.add(drone.id)

            if moves:
                moves.sort(key=lambda m: m[0])
                print(" ".join(text for _drone_id, text in moves))
