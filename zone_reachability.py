class ZoneReachability:

    def __init__(self, data):

        self.hubs, _, _, self.neighbours = data

    def check_all_zones_reachable(self):

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
