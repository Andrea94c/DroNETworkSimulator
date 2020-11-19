
from src.entities.uav_entities import Drone
from src.mac_protocol.depot_mac import DepotMAC

"""
The class is responsable to allocate communication resources to neighbors drones that want to offload data to the depot.
We work over an semplified TDMA approach, each time step only one drone can receive the resource and communicate a packet to the depot. 
"""
class AIDepotMAC(DepotMAC):

    def allocate_resource_to_drone(self, drones : list, cur_step : int) -> Drone:
        """ Return the drone to who allocate bandwith for upload data in this step """
        # implement here your intelligent logic
        return None