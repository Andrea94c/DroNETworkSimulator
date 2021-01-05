
from src.utilities import utilities as util
from src.utilities import config
from src.entities.uav_entities import Drone

"""
The class is responsable to allocate communication resources to neighbors drones that want to offload data to the depot.
We work over an semplified TDMA approach, each time step only one drone can receive the resource and communicate a packet to the depot. 
"""
class DistributedMAC():

    def __init__(self, drone, simulator):
        self.simulator = simulator
        self.drone = drone
        self.print_stats = config.MAC_PRINT_STATS
        self.last_feedback = None

    def communicate(self, cur_step : int) -> bool:
        """ Return the True if the drone should communicate in this slot, False otherwise """
        #return True or False
        return None

    def feedback(self, feedback : bool, packet):
        """
        The method is called automatically to notify the status of tha last packet delivered, for simplicity
            we add also the referred packet in the feedback.

        :param feedback: True if last packet was delivery succesfully, False otherwise
        :param packet: the referred packet
        :return:
        """
        if self.print_stats:
            print(packet, feedback)
        pass

    def run(self, cur_step : int):
        """ run the mac and allocate bandwidth to a particual drone """
        packets_to_send = self.drone.all_packets   # the packets are ordered, from oldest to newest
        communicate = self.communicate(cur_step)  # whether communicate or not

        pck = None
        # pck <- packets_to_send.pick_packet()
        # E.g., :
        # oldest_packet = packets_to_send[0]
        # newest_packet = packets_to_send[-1]

        if communicate:
            self.simulator.depot.receive(self.drone, pck)
            if self.print_stats:
                print("Transmission for drone: ", self.drone.identifier, " to depot, ",
                      len(packets_to_send), " in the buffer.")
