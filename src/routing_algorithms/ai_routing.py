
import numpy as np
from src.utilities import utilities as util
from src.routing_algorithms.BASE_routing import BASE_routing
from matplotlib import pyplot as plt

class AIRouting(BASE_routing):
    def __init__(self, drone, simulator):
        BASE_routing.__init__(self, drone, simulator)
        # random generator
        self.rnd_for_routing_ai = np.random.RandomState(self.simulator.seed)
        self.taken_actions = {}  #id event : (old_state, old_action)

    def feedback(self, drone, id_event, delay, outcome):
        """ return a possible feedback, if the destination drone has received the packet """
        # Packets that we delivered and still need a feedback
        print(self.drone.identifier, self.taken_actions)

        # outcome == -1 if the packet/event expired; 0 if the packets has been delivered to the depot
        # Feedback from a delivered or expired packet
        print(self.drone.identifier, drone, id_event, delay, outcome)

        # remove the entry, the action has received the feedback
        # Be aware, due to network errors we can give the same event to multiple drones and receive multiple feedback for the same packet!!
        if id_event in self.taken_actions:
            state, action = self.taken_actions[id_event]
            del self.taken_actions[id_event]
            # reward or update using the old state and the selected action at that time
            # do something or train the model (?)

    def relay_selection(self, opt_neighbors, pkd):
        """ arg min score  -> geographical approach, take the drone closest to the depot """

        # Only if you need!
        # cell_index = util.TraversedCells.coord_to_cell(size_cell=self.simulator.prob_size_cell,
        #                                                width_area=self.simulator.env_width,
        #                                                x_pos=self.drone.coords[0],  # e.g. 1500
        #                                                y_pos=self.drone.coords[1])[0]  # e.g. 500
        # print(cell_index)
        state, action = None, None

        # Store your current action --- you can add several stuff if needed to take a reward later
        self.taken_actions[pkd.event_ref.identifier] = (state, action)

        return None  # here you should return a drone object!
