

from src.utilities import utilities as util
from src.utilities import config
from src.entities.uav_entities import Drone

"""
The class is responsable to allocate communication resources to neighbors drones that want to offload data to the depot.
We work over an semplified TDMA approach, each time step only one drone can receive the resource and communicate a packet to the depot. 
"""
class DepotMAC():

    def __init__(self, simulator, depot):
        self.simulator = simulator
        self.depot = depot
        self.last_feedback = None
        self.print_stats = config.MAC_PRINT_STATS

    def allocate_resource_to_drone(self, drones : list, cur_step : int) -> Drone:
        """ Return the drone to who allocate bandwith for upload data in this step """
        # implement here your logic
        pass

    def run(self, cur_step : int):
        """ run the mac and allocate bandwidth to a particual drone """
        neigh_drones = self.neighbor_drones()
        drone_to_allocate = self.allocate_resource_to_drone(neigh_drones, cur_step)
        if drone_to_allocate is None:
            return
        self.last_feedback = self.transfer_to_depot(drone_to_allocate, cur_step)
        if self.print_stats:
            print("Transmission for drone: ", self.last_feedback[0].identifier, " was ",
                  self.last_feedback[1], "- with feedback", self.last_feedback[2])

    def transfer_to_depot(self, drone, cur_step) -> tuple:
        """ eventually try to let the drone offload data

            return (chosen_drone, transmission or not, feedback of transmission)

            transmission or not:
                    False : if the drone has not packets to send
                    True :  if the drone has a packet to send right now

            feedback of transmission:
                    0 : if the drone have no lost packets since last update (no packet has been generated since last communication)
                    a positive integer (+pos) : if the drone have some lost packets. The "pos" is the number of packets lost since last communication
                        A packet generated and expired counts as 1.

            E.g.,
                Drone A generates a new (1) packet at time t_0 and we query it immediately.
                        The method returns (A, True, 0) (transmission success and no packets lost since last query)

                Then, drone A generates new nine (9) packets up to time t_1, when we query it.
                At time t_1 it offload the last packet but the remaning (8) packets are lost as expired (not queried in time).
                    The method returns (A, True, 8) (good query, new packet, but we lost 8 packets, we should query it more frequently).

                Then, drone A generates 4 packets up to time t_2. They expire before we query it at time t_3.
                thus, at time t_3 the drone A has not alive packets and we query it.
                    The method return (A, False, 4) (bad query, no new packet, and we also lost 4 packets, we should query it more frequently).

                Then, drone A does not generate packets anymore. At time t_4 the drone A has not packets and we query it.
                    The method return (A, False, 0) (no new packets, useless query).

        """
        drone_packets = drone.all_packets()
        drop_packet_since_last_upload = drone.drop_packet_since_last_upload
        drone.drop_packet_since_last_upload = 0

        # no recent packet to send
        if len(drone_packets) <= 0:
            return drone, False, drop_packet_since_last_upload

        # send a packet
        packet_to_offload = drone_packets[-1]  # take the most recent
        self.depot.add_packet(packet_to_offload, cur_step)

        # add metrics: all the packets notified to the depot
        self.simulator.metrics.delivered_packet_for_drone[drone.identifier] += 1

        # remove this packet from the drone buffer
        drone.remove_packet(packet_to_offload)
        return drone, True, drop_packet_since_last_upload

    def neighbor_drones(self) -> list:
        """ return a list of drones that want to communicate with the depot """
        neigh_drones = []
        for drone in self.simulator.drones:
            drones_distance = util.euclidean_distance(self.depot.coords, drone.coords)
            if drones_distance <= min(drone.communication_range,
                                          self.depot.communication_range):  # one feels the other
                neigh_drones.append(drone)

        return neigh_drones

