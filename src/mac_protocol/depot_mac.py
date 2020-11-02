

from src.utilities import utilities as util
from src.entities.uav_entities import Drone

"""
The class is responsable to allocate communication resources to neighbors drones that want to offload data to the depot.
We work over an semplified TDMA approach, each time step only one drone can receive the resource and communicate a packet to the depot. 
"""
class DepotMAC():

    def __init__(self, simulator, depot):
        self.simulator = simulator
        self.depot = depot

    def allocate_resource_to_drone(self, drones : list, cur_step : int) -> Drone:
        """ Return the drone to who allocate bandwith for upload data in this step """
        # implement here your logic
        pass

    def run(self, cur_step : int):
        """ run the mac and allocate bandwidth to a particual drone """
        neigh_drones = self.neighbor_drones()
        drone_to_allocate = self.allocate_resource_to_drone(neigh_drones, cur_step)
        feedback = self.transfer_to_depot(drone_to_allocate, cur_step)
        print("Transmission for drone: ", drone_to_allocate.identifier, " has feedback: ", feedback)

    def transfer_to_depot(self, drone, cur_step):
        """ eventually try to let the drone offload data

            return: - -1 : if the drone has not new packets to send;
                    - pos : if the drone had sent the last packet. The "pos" is the number of packets lost (since last upload)

            E.g.,
                Drone A generates a new (1) packet at time t and we query it immediately. The method returns 0 (success and no packets lost)
                Drone A generates new nine (9) packets up to t', when we query it.
                At time t' it offload the last packet but the remaning (8) packets are lost as not queried in time. The method return 8.
                At time t'' the drone A has not packets. We query it and the method return -1 (no new packets, usefull query).
        """
        drone_packets = drone.all_packets()
        if len(drone_packets) <= 0:
            return -1
        packet_to_offload = drone_packets[-1]  # take the most recent

        drop_packet_since_last_upload = drone.drop_packet_since_last_upload
        self.depot.add_packet(packet_to_offload, cur_step)

        # add metrics: all the packets notified to the depot
        self.simulator.metrics.delivered_packet_for_drone[drone.identifier] += 1

        # remove this packet from the drone buffer
        drone.remove_packet(packet_to_offload)
        return drop_packet_since_last_upload

    def neighbor_drones(self) -> list:
        """ return a list of drones that want to communicate with the depot """
        neigh_drones = []
        for drone in self.simulator.drones:
            drones_distance = util.euclidean_distance(self.depot.coords, drone.coords)
            if drones_distance <= min(drone.communication_range,
                                          self.depot.communication_range):  # one feels the other
                neigh_drones.append(drone)

        return neigh_drones

