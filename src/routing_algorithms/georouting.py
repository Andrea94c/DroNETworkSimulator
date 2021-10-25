
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


"""
Mean number of relays:  1.0706168831168832
number_of_generated_events 231
number_of_detected_events 231
all_control_packets_in_simulation 75672
all_data_packets_in_simulation 1134
number_of_events_to_depot 76
number_of_packets_to_depot 78
packet_mean_delivery_time 936.8333333333334
event_mean_delivery_time 916.1973684210526
"""