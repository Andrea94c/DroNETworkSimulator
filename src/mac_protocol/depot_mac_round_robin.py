from src.entities.uav_entities import Drone
from src.mac_protocol.depot_mac import DepotMAC

"""
Details:
An example of a random Depot MAC protocol. Here we use simple a round robin methodology.
"""


class RoundRobinDepotMAC(DepotMAC):

    def __init__(self, simulator, depot):
        DepotMAC.__init__(self, simulator, depot)
        self.last_choice = -1

    def allocate_resource_to_drone(self, drones: list, cur_step: int) -> Drone:
        """
        This method select the drone to query using round-robin
        @param drones: A list of Drones object
        @param cur_step: an integer value that represent current time step
        @return: Returns the drone to query in this step
        """
        # implement here your logic
        # schedule each time the next drone, like a round-robin approach
        if self.last_choice >= self.simulator.n_drones - 1:
            self.last_choice = -1

        # increment the last_choice to the next drone to select
        self.last_choice += 1

        # select the drone
        return drones[self.last_choice]
