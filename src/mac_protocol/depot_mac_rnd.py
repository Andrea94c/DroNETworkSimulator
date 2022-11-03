from src.entities.uav_entities import Drone
from src.mac_protocol.depot_mac import DepotMAC

"""
Details:
An example of a random Depot MAC protocol. Here we do not have any kind of logic behind. We only choose a random
action (a drone to query) among the available actions. This approach as expected does not work very well.
"""


class RandomDepotMAC(DepotMAC):

    def allocate_resource_to_drone(self, drones: list, cur_step: int) -> Drone:
        """
        This method select the drone to query randomly
        @param drones: A list of Drones object
        @param cur_step: an integer value that represent current time step
        @return: Returns the drone to query in this step
        """

        return self.simulator.rnd_routing.choice(drones)
