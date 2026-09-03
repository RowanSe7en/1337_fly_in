class ZoneReachability:
    """Validate that all zones are reachable from the start hub."""

    def __init__(self, data):
        """Initialize the reachability validator.

        Args:
            data: Parsed simulation data containing hub information and
                neighbour relationships.
        """
        self.hubs, _, _, self.neighbours = data

    def check_all_zones_reachable(self):
        """Check whether every zone can be reached from the start hub.

        Traverses the zone graph starting from the start hub and raises an
        exception if one or more zones cannot be reached.

        Raises:
            ValueError: If one or more zones are unreachable from the
                start hub.

        Returns:
            None: This method returns nothing when all zones are reachable.
        """
        start_zone = self.hubs["start_hub"]["name"]
        visited = set()
        stack = [start_zone]

        while stack:
            zone = stack.pop()

            if zone in visited:
                continue

            visited.add(zone)

            for neighbour in self.neighbours.get(zone, []):
                if neighbour not in visited:
                    stack.append(neighbour)

        all_zones = {
            hub_data["name"]
            for hub_data in self.hubs.values()
        }

        unreachable_zones = all_zones - visited

        if unreachable_zones:
            raise ValueError(
                f"Unreachable zones: {', '.join(sorted(unreachable_zones))}"
            )
