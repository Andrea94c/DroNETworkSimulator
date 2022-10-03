
import numpy as np
import src.utilities.utilities as util

from src.routing_algorithms.BaseRouting import BaseRouting

class AndreaGeoRouting(BaseRouting):

    def __init__(self, drone, simulator):
        BaseRouting.__init__(self, simulator, drone)

    def relay_selection(self, neighbors):
        """ arg min score  -> geographical approach, take the drone closest to the depot """
        #if self.drone.identifier == 0:
        #    print(opt_neighbors)

        depot_pos = self.drone.depot.coords
        cur_step = self.simulator.cur_step
        my_distance_from_bs = util.euclidean_distance(self.drone.coords, depot_pos)
        drone_to_send = None
        current_score = my_distance_from_bs
        for hll_pck, close_drone in neighbors:
            close_drone_pos = hll_pck.current_position
            close_drone_speed = hll_pck.speed  # m / s
            hll_pck_timestep_creation = hll_pck.time_step_creation
            next_target_close_drone = hll_pck.next_target

            # util.euclidean_distance( --> meters
            #delta_timesteps = None
            #delta_seconds = delta_timesteps * self.simulator.time_step_duration
            distance_close_drone_to_bs = util.euclidean_distance(close_drone_pos,
                                                                 depot_pos)

            if (distance_close_drone_to_bs < current_score):
                drone_to_send = close_drone
                current_score = distance_close_drone_to_bs

        return drone_to_send