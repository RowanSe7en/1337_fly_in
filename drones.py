class Drone:
    """Represent a drone and its assigned simulation path."""

    def __init__(self, id):

        self.id = id
        self.path = []
        self.start_turn = -1


drones_list = []


def drone_creator(nb_drones):
    """Create the requested drones."""
    for drone_id in range(1, nb_drones + 1):
        drones_list.append(Drone(drone_id))
