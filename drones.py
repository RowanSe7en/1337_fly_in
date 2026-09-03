from typing import List


class Drone:
    """Represent a drone and its assigned simulation path."""

    def __init__(self, id: int) -> None:
        """Initialize a drone.

        Args:
            id: Unique identifier assigned to the drone.
        """
        self.id: int = id
        self.path: List[str] = []
        self.start_turn: int = -1


drones_list: List[Drone] = []


def drone_creator(nb_drones: int) -> None:
    """Create and store the requested number of drones.

    Args:
        nb_drones: Number of drones to create.

    Returns:
        None: The created drones are appended to the global
    """
    for drone_id in range(1, nb_drones + 1):
        drones_list.append(Drone(drone_id))
