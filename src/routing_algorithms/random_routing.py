import src.utilities.utilities as util
from src.routing_algorithms.BaseRouting import BaseRouting


class RandomRouting(BaseRouting):

    def __init__(self, drone, simulator):
        BaseRouting.__init__(self, simulator, drone)

    def relay_selection(self, neighbors):
        """ arg min score  -> geographical approach, take the drone closest to the depot """
        # opt_neighbors --> [(hck packet : drone istance), (hck packet, drone istance).. .. ]
        return self.simulator.rnd_routing.choice([v[1] for v in neighbors])
