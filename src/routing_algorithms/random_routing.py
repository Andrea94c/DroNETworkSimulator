
import src.utilities.utilities as util
from src.routing_algorithms.BASE_routing import BASE_routing

class RandomRouting(BASE_routing):

    def __init__(self, drone, simulator):
        BASE_routing.__init__(self, drone, simulator)

    def relay_selection(self, opt_neighbors, pkd):
        """ arg min score  -> geographical approach, take the drone closest to the depot """
        # opt_neighbors --> [(hck packet : drone istance), (hck packet, drone istance).. .. ]
        if len(opt_neighbors) == 0:
            return None
        return self.simulator.rnd_routing.choice([v[1] for v in opt_neighbors])

