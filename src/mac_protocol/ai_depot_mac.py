from src.entities.uav_entities import Drone
from src.mac_protocol.depot_mac import DepotMAC

"""
PLEASE write here your solution. 
"""


class BanditDepotMAC(DepotMAC):

    def __init__(self, simulator, depot):
        super().__init__(simulator, depot)

    def allocate_resource_to_drone(self, drones: list, cur_step: int) -> Drone:
        """
        This method selects the drone to query using multi-armed bandit
        @param drones: A list of Drones object
        @param cur_step: an integer value that represent current time step
        @return: Returns the drone to query in this step
        """

        # implement here your intelligent logic
        return None
