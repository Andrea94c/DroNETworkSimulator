
from src.entities.uav_entities import Drone
from src.mac_protocol.depot_mac import DepotMAC

"""
The class is responsable to allocate communication resources to neighbors drones that want to offload data to the depot.
We work over an semplified TDMA approach, each time step only one drone can receive the resource and communicate a packet to the depot. 
"""
class RoundRobinDepotMAC(DepotMAC):

    def __init__(self, simulator, depot):
        DepotMAC.__init__(self, simulator, depot)
        self.last_choice = -1

    def allocate_resource_to_drone(self, drones : list, cur_step : int) -> Drone:
        """ Return the drone to who allocate bandwith for upload data in this step """
        # implement here your logic
        # schedule each time the next drone, like a round robin approach
        if self.last_choice >= self.simulator.n_drones - 1:
            self.last_choice = -1

        # increment the last_choice to the next drone to select
        self.last_choice += 1

        # select the drone
        return drones[self.last_choice]
