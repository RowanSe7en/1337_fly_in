import math
import heapq
from drones import drones_list


class Algo:

    def __init__(self, data):

        self.data = data

        (
            self.hubs,
            self.connections,
            self.nb_drones,
            self.neighbours,
        ) = self.data

        self.start_hub = self.hubs['start_hub']['name']
        self.end_hub = self.hubs['end_hub']['name']

        self.unvisited = [v['name'] for v in self.hubs.values()]

        self.cost = 0
        self.all_paths = []

        self.turnes = []
        self.zones_at_turnes = []

    def shortest_path_search(
        self,
        start_hub,
        end_hub,
        unvisited_list=None,
        exclude_zones=None,
        allowed_zones=None,
        skip_blocked=True,
        from_placeholder="none",
    ):

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

        while initiate_neighbours:

            for zone in initiate_neighbours:
                is_blocked = 0
                if (
                    zone in unvisited_list
                    and (exclude_zones is None or zone not in exclude_zones)
                ):
                    for hub_data in self.hubs.values():
                        if hub_data['name'] == zone:
                            if hub_data['metadata']['zone'] == "blocked":
                                if skip_blocked:
                                    is_blocked = 1
                                break
                            elif hub_data['metadata']['zone'] in (
                                "normal",
                                "priority",
                            ):
                                cost = 1
                                break
                            elif hub_data['metadata']['zone'] == "restricted":
                                cost = 2
                                break

                    if is_blocked:
                        continue

                    if allowed_zones is None:
                        if zone in path_dict:
                            if (
                                path_dict[zone][0]
                                > path_dict[from_hub][0] + cost
                            ):
                                path_dict[zone] = (
                                    path_dict[from_hub][0] + cost,
                                    from_hub,
                                )
                        else:
                            path_dict[zone] = (
                                path_dict[from_hub][0] + cost,
                                from_hub,
                            )
                    else:
                        if zone in path_dict and zone in allowed_zones:
                            if (
                                path_dict[zone][0]
                                > path_dict[from_hub][0] + cost
                            ):
                                path_dict[zone] = (
                                    path_dict[from_hub][0] + cost,
                                    from_hub,
                                )
                        else:
                            if zone in allowed_zones:
                                path_dict[zone] = (
                                    path_dict[from_hub][0] + cost,
                                    from_hub,
                                )

            heap = []
            for zone, (zone_cost, zone_from_hub) in path_dict.items():
                if zone in unvisited_list:
                    heapq.heappush(heap, (zone_cost, zone_from_hub, zone))

            _, _, lower_cost_zone_name = heapq.heappop(heap)
            unvisited_list.remove(lower_cost_zone_name)
            from_hub = lower_cost_zone_name

            if not lower_cost_zone_name or lower_cost_zone_name == end_hub:
                break
            else:
                initiate_neighbours = list(
                    self.neighbours[lower_cost_zone_name]
                )

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

        (
            self.short_path,
            self.cost,
            self.short_path_dict,
            self.unvisited,
        ) = self.shortest_path_search(
            start_hub=self.start_hub,
            end_hub=self.end_hub,
            unvisited_list=self.unvisited,
        )

        if not math.isfinite(self.short_path_dict[self.end_hub][0]):
            raise ValueError("No solution to the map")

    def get_zone_type(self, zone):

        for v in self.hubs.values():
            if v['name'] == zone:
                return v['metadata']['zone']

        return None

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

    def update_future_zone_turns(
        self,
        drone,
        changed_turn,
        old_zone,
        new_zone,
    ):

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

    def _has_capacity_for_search(self, zone, turn):

        if zone in (self.start_hub, self.end_hub):
            return True

        self.ensure_zone_turn(turn)
        current_drones = len(self.zones_at_turnes[turn][1][zone])

        for hub in self.hubs.values():
            if hub["name"] == zone:
                return current_drones < hub["metadata"]["max_drones"]

        return False

    def find_drone_path(self, start_turn):

        start = self.start_hub
        end = self.end_hub

        pq = [(start_turn, start, [])]
        visited = set()

        while pq:

            turn, zone, path = heapq.heappop(pq)

            if zone == end:
                return path

            if (zone, turn) in visited:
                continue
            visited.add((zone, turn))

            if self._has_capacity_for_search(zone, turn + 1):
                state = (zone, turn + 1)
                if state not in visited:
                    heapq.heappush(pq, (turn + 1, zone, path + [zone]))

            for neighbour in self.neighbours.get(zone, []):
                zone_type = self.get_zone_type(neighbour)
                if zone_type == "blocked":
                    continue

                if not self._has_capacity_for_search(neighbour, turn + 1):
                    continue

                stay = 2 if zone_type == "restricted" else 1
                if (
                    stay == 2
                    and not self._has_capacity_for_search(neighbour, turn + 2)
                ):
                    continue

                new_turn = turn + stay
                state = (neighbour, new_turn)
                if state not in visited:
                    heapq.heappush(
                        pq, (new_turn, neighbour, path + [neighbour] * stay)
                    )

        raise ValueError("No valid path exists between start and end zones")

    def commit_path(self, drone, path, start_turn):

        turn = start_turn

        for zone in path:
            turn += 1

            if len(self.turnes) < turn:
                self.turnes.append((
                    turn,
                    [
                        n['name'] for n in self.hubs.values()
                        if n['name'] not in [self.start_hub]
                    ]
                ))
            self.ensure_zone_turn(turn)

            drone.path.append(zone)

            if zone != self.end_hub and not self.zone_has_capacity(zone, turn):
                self.turnes[turn - 1][1].remove(zone)
            self.move_drone(drone, zone, turn, "search")

        drone.start_turn = start_turn + 1

    def ecah_drone_path_assigner(self):

        self.turnes = [
            (
                1,
                [
                    hub['name']
                    for hub in self.hubs.values()
                    if hub['name'] not in [self.start_hub]
                ],
            )
        ]
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

        for drone in drones_list:

            path = self.find_drone_path(start_turn=0)
            self.commit_path(drone, path, start_turn=0)

            can_i_add = 1
            for e in self.all_paths:
                if drone.path == e[1]:
                    can_i_add = 0
                    break
            if can_i_add:
                self.all_paths.append([len(drone.path) + 1, drone.path])

        return self.zones_at_turnes
