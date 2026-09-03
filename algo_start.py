import math
import heapq
from typing import Any, Dict, List, Optional, Set, Tuple, cast
from custom_types import ParsedData
from drones import Drone, drones_list


class Algo:
    """Calculate and assign paths for drones in the Fly-in simulation."""

    def __init__(self, data: ParsedData) -> None:
        """Initialize the path-finding algorithm.

        Args:
            data: Parsed simulation data containing hubs, connections,
                number of drones, and neighbour relationships.
        """
        self.data = data
        (
            self.hubs,
            self.connections,
            self.nb_drones,
            self.neighbours,
        ) = self.data

        self.start_hub: str = self.hubs['start_hub']['name']
        self.end_hub: str = self.hubs['end_hub']['name']
        self.unvisited: List[str] = [
            v['name'] for v in self.hubs.values()
        ]
        self.cost: float = 0
        self.all_paths: List[List[Any]] = []
        self.turnes: List[Tuple[int, List[str]]] = []
        self.zones_at_turnes: List[Tuple[int, Dict[str, List[int]]]] = []

    def shortest_path_search(
        self,
        start_hub: str,
        end_hub: str,
        unvisited_list: Optional[List[str]] = None,
        exclude_zones: Optional[List[str]] = None,
        allowed_zones: Optional[List[str]] = None,
        skip_blocked: bool = True,
        from_placeholder: str = "none",
    ) -> Tuple[List[str], float, Dict[str, Tuple[float, str]], List[str]]:
        """Find the shortest path between two hubs.

        Uses a weighted graph traversal where normal and priority zones
        have a cost of one turn and restricted zones have a cost of two
        turns.

        Args:
            start_hub: Name of the hub where the search starts.
            end_hub: Name of the destination hub.
            unvisited_list: List of hubs that may still be visited.
            exclude_zones: Optional collection of zones excluded from the
                search.
            allowed_zones: Optional collection restricting which zones
                may be considered.
            skip_blocked: Whether blocked zones should be excluded.
            from_placeholder: Initial predecessor value for the start hub.

        Returns:
            tuple: A tuple containing the shortest path, its total cost,
            the path dictionary, and the remaining unvisited hubs.
        """
        if unvisited_list is None:
            unvisited_list = [v['name'] for v in self.hubs.values()]

        path_dict: Dict[str, Tuple[float, str]] = {}

        for e in unvisited_list:
            path_dict[e] = (math.inf, "none")

        from_hub = start_hub
        path_dict[from_hub] = (0, from_placeholder)
        initiate_neighbours = list(self.neighbours[from_hub])
        cost: float = 0
        unvisited_list.remove(from_hub)

        while initiate_neighbours:
            for zone in initiate_neighbours:
                is_blocked = 0

                if (
                    zone in unvisited_list
                    and (
                        exclude_zones is None
                        or zone not in exclude_zones
                    )
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

            heap: List[Tuple[float, str, str]] = []

            for zone, (zone_cost, zone_from_hub) in path_dict.items():
                if zone in unvisited_list:
                    heapq.heappush(
                        heap,
                        (zone_cost, zone_from_hub, zone)
                    )

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

        path: List[str] = []
        a = end_hub
        path.append(a)

        while a != start_hub:
            path.append(path_dict[a][1])
            a = path_dict[a][1]

        path.reverse()
        path.remove(start_hub)

        return path, path_dict[end_hub][0], path_dict, unvisited_list

    def find_the_shortest_path(self) -> None:
        """Find and store the shortest path from start to end hub.

        Updates the algorithm's shortest path, path cost, path dictionary,
        and unvisited hub list. Raises an error when no valid path exists.

        Raises:
            ValueError: If no valid path exists between the start and
                end hubs.

        Returns:
            None: The calculated path information is stored on the
            instance.
        """
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

    def get_zone_type(self, zone: str) -> Optional[str]:
        """Return the type of a specified zone.

        Args:
            zone: Name of the zone whose type should be retrieved.

        Returns:
            str | None: The zone type, or None if the zone is not found.
        """
        for v in self.hubs.values():
            if v['name'] == zone:
                return cast(str, v['metadata']['zone'])

        return None

    def ensure_zone_turn(self, turn: int) -> None:
        """Ensure that a simulation snapshot exists for a given turn.

        Missing snapshots are created by copying the previous turn's
        zone-to-drone mapping.

        Args:
            turn: Simulation turn that must exist.

        Returns:
            None: The required snapshots are added to the instance.
        """
        while len(self.zones_at_turnes) <= turn:
            previous = self.zones_at_turnes[-1][1]
            next_turn = len(self.zones_at_turnes)

            self.zones_at_turnes.append((
                next_turn,
                {
                    zone: list(drone_ids)
                    for zone, drone_ids in previous.items()
                }
            ))

    def drone_at_which_zone(
        self,
        drone: Drone,
        snapshot: Dict[str, List[int]],
    ) -> Optional[str]:
        """Return the zone containing a specified drone.

        Args:
            drone: Drone whose current zone is being searched.
            snapshot: Mapping of zone names to the drone IDs they contain.

        Returns:
            str | None: The zone containing the drone, or None if the
            drone is not present in the snapshot.
        """
        for zone_name, drone_ids in snapshot.items():
            if drone.id in drone_ids:
                return zone_name

        return None

    def update_future_zone_turns(
        self,
        drone: Drone,
        changed_turn: int,
        old_zone: Optional[str],
        new_zone: str,
    ) -> None:
        """Update future snapshots after moving a drone.

        Removes the drone from its old zone and places it in the new zone
        in subsequent snapshots until its recorded position changes.

        Args:
            drone: Drone whose future position must be updated.
            changed_turn: Turn at which the drone changes zones.
            old_zone: Zone the drone is leaving.
            new_zone: Zone the drone is entering.

        Returns:
            None: Future simulation snapshots are updated in place.
        """
        if old_zone == new_zone or old_zone is None:
            return

        for t in range(
            changed_turn + 1,
            len(self.zones_at_turnes)
        ):
            snapshot = self.zones_at_turnes[t][1]

            if self.drone_at_which_zone(drone, snapshot) != old_zone:
                break

            for drone_ids in snapshot.values():
                if drone.id in drone_ids:
                    drone_ids.remove(drone.id)

            if drone.id not in snapshot[new_zone]:
                snapshot[new_zone].append(drone.id)

    def move_drone(
        self,
        drone: Drone,
        zone: str,
        turn: int,
        ff: str,
    ) -> None:
        """Move a drone to a specified zone at a simulation turn.

        Updates the current snapshot, handles zone capacity tracking,
        removes the drone from its previous zone, and propagates the
        movement to relevant future snapshots.

        Args:
            drone: Drone being moved.
            zone: Destination zone.
            turn: Simulation turn at which the movement occurs.
            ff: Movement source or mode identifier.

        Returns:
            None: The simulation snapshots are updated in place.
        """
        self.ensure_zone_turn(turn)

        zones = self.zones_at_turnes[turn][1]
        old_zone = self.drone_at_which_zone(drone, zones)

        for v in self.hubs.values():
            if old_zone is not None and v["name"] == old_zone:
                max_drones = v["metadata"]["max_drones"]

                if (
                    len(self.zones_at_turnes[turn][1][old_zone])
                    < max_drones
                ):
                    if old_zone not in self.turnes[turn - 1][1]:
                        self.turnes[turn - 1][1].append(old_zone)
                break

        for drone_ids in zones.values():
            if drone.id in drone_ids:
                drone_ids.remove(drone.id)

        if drone.id not in zones[zone]:
            zones[zone].append(drone.id)

        self.update_future_zone_turns(
            drone,
            turn,
            old_zone,
            zone
        )

    def zone_has_capacity(self, zone: str, turn: int) -> bool:
        """Check whether a zone can accept another drone.

        The end hub always has capacity. Other zones are checked against
        their configured maximum drone capacity.

        Args:
            zone: Zone whose capacity should be checked.
            turn: Simulation turn at which capacity is evaluated.

        Returns:
            bool: True if the zone can accept another drone, otherwise
            False.
        """
        if zone == self.end_hub:
            return True

        self.ensure_zone_turn(turn)

        current_drones = len(
            self.zones_at_turnes[turn][1][zone]
        )

        for hub in self.hubs.values():
            if hub["name"] == zone:
                max_drones = hub["metadata"]["max_drones"]
                return bool(current_drones + 1 < max_drones)

        return False

    def _has_capacity_for_search(self, zone: str, turn: int) -> bool:
        """Check zone capacity while searching for a path.

        Start and end hubs are treated as having unlimited capacity.
        Other zones are checked against their configured maximum capacity.

        Args:
            zone: Zone whose capacity should be checked.
            turn: Simulation turn at which capacity is evaluated.

        Returns:
            bool: True if the zone has available capacity, otherwise
            False.
        """
        if zone in (self.start_hub, self.end_hub):
            return True

        self.ensure_zone_turn(turn)

        current_drones = len(
            self.zones_at_turnes[turn][1][zone]
        )

        for hub in self.hubs.values():
            if hub["name"] == zone:
                return bool(current_drones < hub["metadata"]["max_drones"])

        return False

    def find_drone_path(self, start_turn: int) -> List[str]:
        """Find a valid time-aware path for a drone.

        Searches through zone and turn states while accounting for zone
        capacity, blocked zones, and the additional movement cost of
        restricted zones.

        Args:
            start_turn: Simulation turn from which the search begins.

        Returns:
            list: Sequence of zones representing the drone's path.

        Raises:
            ValueError: If no valid path exists between the start and end
                zones.
        """
        start = self.start_hub
        end = self.end_hub
        pq: List[Tuple[int, str, List[str]]] = [(start_turn, start, [])]
        visited: Set[Tuple[str, int]] = set()

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
                    heapq.heappush(
                        pq,
                        (turn + 1, zone, path + [zone])
                    )

            for neighbour in self.neighbours.get(zone, []):
                zone_type = self.get_zone_type(neighbour)

                if zone_type == "blocked":
                    continue

                if not self._has_capacity_for_search(
                    neighbour,
                    turn + 1
                ):
                    continue

                stay = 2 if zone_type == "restricted" else 1

                if (
                    stay == 2
                    and not self._has_capacity_for_search(
                        neighbour,
                        turn + 2
                    )
                ):
                    continue

                new_turn = turn + stay
                state = (neighbour, new_turn)

                if state not in visited:
                    heapq.heappush(
                        pq,
                        (
                            new_turn,
                            neighbour,
                            path + [neighbour] * stay
                        )
                    )

        raise ValueError(
            "No valid path exists between start and end zones"
        )

    def commit_path(
        self,
        drone: Drone,
        path: List[str],
        start_turn: int,
    ) -> None:
        """Commit a calculated path to the simulation.

        Adds the drone's movements to the appropriate simulation turns,
        tracks zone capacity, and updates the drone's assigned path.

        Args:
            drone: Drone receiving the calculated path.
            path: Sequence of zones the drone should traverse.
            start_turn: Simulation turn at which path assignment begins.

        Returns:
            None: The drone path and simulation snapshots are updated
            in place.
        """
        turn = start_turn

        for zone in path:
            turn += 1

            if len(self.turnes) < turn:
                self.turnes.append((
                    turn,
                    [
                        n['name']
                        for n in self.hubs.values()
                        if n['name'] not in [self.start_hub]
                    ]
                ))

            self.ensure_zone_turn(turn)
            drone.path.append(zone)

            if (
                zone != self.end_hub
                and not self.zone_has_capacity(zone, turn)
            ):
                self.turnes[turn - 1][1].remove(zone)

            self.move_drone(
                drone,
                zone,
                turn,
                "search"
            )

        drone.start_turn = start_turn + 1

    def ecah_drone_path_assigner(
        self,
    ) -> List[Tuple[int, Dict[str, List[int]]]]:
        """Assign valid paths to all drones in the simulation.

        Initializes the simulation state, calculates a path for each
        drone, commits each path to the simulation, and records unique
        paths for later reference.

        Returns:
            list: Simulation snapshots containing the drone positions
            for each simulation turn.
        """
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
                        [
                            drone.id
                            for drone in drones_list
                        ]
                        if hub_data["name"] == self.start_hub
                        else []
                    )
                    for hub_data in self.hubs.values()
                }
            )
        ]

        for drone in drones_list:
            path = self.find_drone_path(start_turn=0)
            self.commit_path(
                drone,
                path,
                start_turn=0
            )

            can_i_add = 1

            for e in self.all_paths:
                if drone.path == e[1]:
                    can_i_add = 0
                    break

            if can_i_add:
                self.all_paths.append([
                    len(drone.path) + 1,
                    drone.path
                ])

        return self.zones_at_turnes
