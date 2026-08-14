class Drone:

    def __init__(self, id):

        self.id = id
        self.path = []
        self.start_turn = -1 

drones_list = []

def drone_creator(nb_drones):

    for d in range(1, nb_drones + 1):
        drones_list.append(Drone(d))

