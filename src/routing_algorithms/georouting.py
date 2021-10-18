
import numpy as np
import src.utilities.utilities as util

from src.routing_algorithms.BASE_routing import BASE_routing

class GeoRouting(BASE_routing):
    def __init__(self, drone, simulator):
        BASE_routing.__init__(self, drone, simulator)

    def relay_selection(self, opt_neighbors):
        """ arg min score  -> geographical approach, take the drone closest to the depot """
        # TODO
        return None