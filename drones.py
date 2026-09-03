class Drone:
    """Represent a drone and its assigned simulation path."""

    def __init__(self, id):
        """Initialize a drone.

        Args:
            id: Unique identifier assigned to the drone.
        """
        self.id = id
        self.path = []
        self.start_turn = -1


drones_list = []


def drone_creator(nb_drones):
    """Create and store the requested number of drones.

    Args:
        nb_drones: Number of drones to create.

    Returns:
        None: The created drones are appended to the global
    """
    for drone_id in range(1, nb_drones + 1):
        drones_list.append(Drone(drone_id))
