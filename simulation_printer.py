class SimulationPrinter:
    """Format and print movements represented by simulation snapshots."""

    def __init__(
        self,
        zones_at_turnes,
        drones_list,
        end_hub,
        get_zone_type,
    ):
        """Initialize the simulation printer.

        Args:
            zones_at_turnes: Simulation snapshots containing the zones and
                drones present at each turn.
            drones_list: List of drones participating in the simulation.
            end_hub: Name of the destination hub where drones are delivered.
            get_zone_type: Callable used to retrieve the type of a zone.
        """
        self.zones_at_turnes = zones_at_turnes
        self.drones_list = drones_list
        self.end_hub = end_hub
        self.get_zone_type = get_zone_type

    def _drone_at_which_zone(self, drone, snapshot):
        """Return the zone containing a specific drone.

        Args:
            drone: Drone whose current zone is being searched.
            snapshot: Mapping of zone names to the drone IDs currently
                present in each zone.

        Returns:
            str | None: The name of the zone containing the drone, or
            None if the drone is not present in the snapshot.
        """
        for zone, drones in snapshot.items():
            if drone.id in drones:
                return zone

        return None

    def print_simulation(self):
        """Print the drone movements for every simulation turn.

        Tracks each drone between consecutive simulation snapshots and
        prints its movements in ascending drone ID order. Restricted zones
        require an additional turn before the movement is considered
        complete. The method stops when all drones have reached the end hub
        or when no further simulation snapshots are available.

        Returns:
            int: The number of simulation turns containing printed
            movements.
        """
        if not self.zones_at_turnes:
            return 0

        delivered = set()
        pending_restricted_turn = {}
        turns = 0

        for turn in range(1, len(self.zones_at_turnes)):
            if len(delivered) >= len(self.drones_list):
                break

            prev_snapshot = self.zones_at_turnes[turn - 1][1]
            curr_snapshot = self.zones_at_turnes[turn][1]
            moves = []

            for drone in self.drones_list:

                if drone.id in delivered:
                    continue

                prev_zone = self._drone_at_which_zone(
                    drone,
                    prev_snapshot,
                )
                curr_zone = self._drone_at_which_zone(
                    drone,
                    curr_snapshot,
                )

                if curr_zone is None:
                    continue

                if curr_zone == prev_zone:
                    if pending_restricted_turn.get(drone.id) == curr_zone:
                        moves.append((drone.id, f"D{drone.id}-{curr_zone}"))
                        del pending_restricted_turn[drone.id]

                        if curr_zone == self.end_hub:
                            delivered.add(drone.id)

                    continue

                moves.append((drone.id, f"D{drone.id}-{curr_zone}"))

                if self.get_zone_type(curr_zone) == "restricted":
                    pending_restricted_turn[drone.id] = curr_zone
                elif curr_zone == self.end_hub:
                    delivered.add(drone.id)

            if moves:
                moves.sort(key=lambda move: move[0])
                print(" ".join(text for _drone_id, text in moves))
                turns += 1

        return turns
