from src.routing_algorithms.BASE_routing import BASE_routing
from src.utilities import  utilities
import numpy as np


class CloRouting(BASE_routing):

    def relay_selection(self, opt_neighbors, pkd):
        """
        This routing is going to select as relay the next drone which is closest to me.

        :param opt_neighbors: list [(hello_packet, drone_instance), (hello....)]
        :return: drone_instance of the next relay
        """
        best_drone_distance_from_me = np.inf
        best_drone = None

        for hpk, drone_istance in opt_neighbors:
            time_step_creation = hpk.time_step_creation
            cur_pos = hpk.cur_pos
            speed = hpk.speed
            next_target = hpk.next_target

            distance_for_me = utilities.euclidean_distance(cur_pos, self.drone.coords)
            if distance_for_me < best_drone_distance_from_me:
                best_drone_distance_from_me = distance_for_me
                best_drone = drone_istance

        return best_drone

