
import src.utilities.utilities as util
from src.entities.packets.packets import DataPacket
from src.simulation.metrics import Metrics


class MediumDispatcher:

    def __init__(self, metric_class: Metrics):
        self.packets = []
        self.metric_class = metric_class

    def send_packet_to_medium(self, packet, source_drone, destination_drone, to_send_ts):
        """

        @param packet: The Packet to send
        @param source_drone: Source Drone
        @param destination_drone: Destination Drone
        @param to_send_ts:
        """

        if isinstance(packet, DataPacket):
            self.metric_class.all_data_packets_in_simulation += 1
        else:
            self.metric_class.all_control_packets_in_simulation += 1

        self.packets.append((packet, source_drone, destination_drone, to_send_ts))

    def run_medium(self, current_ts):
        """

        @param current_ts:
        @return:
        """
        to_drop_indices = []
        original_self_packets = self.packets[:]
        self.packets = []

        for i in range(len(original_self_packets)):
            packet, src_drone, dst_drone, to_send_ts = original_self_packets[i]

            if to_send_ts == current_ts:  # time to send this packet
                to_drop_indices.append(i)

                if src_drone.identifier != dst_drone.identifier:
                    drones_distance = util.euclidean_distance(src_drone.coords, dst_drone.coords)
                    if drones_distance <= min(src_drone.communication_range, dst_drone.communication_range):
                        if dst_drone.routing_algorithm.channel_success(drones_distance):
                            dst_drone.routing_algorithm.drone_reception(src_drone, packet)  # reception of a packet

        original_self_packets = [original_self_packets[i] for i in range(len(original_self_packets)) if i not in to_drop_indices]
        self.packets = original_self_packets + self.packets
