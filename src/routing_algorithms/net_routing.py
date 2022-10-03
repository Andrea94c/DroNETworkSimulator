import src.utilities.utilities as util
from src.entities.packets.packets import DataPacket
from src.simulation.metrics import Metrics

# TODO: Write Docs


class MediumDispatcher:

    def __init__(self, metric_class: Metrics):
        """

        @param metric_class:
        """
        self.packets = []
        self.metric_class = metric_class

    def send_packet_to_medium(self, packet_to_send, source_drone, destination_drone, to_send_ts):
        """

        @param packet_to_send: The Packet to send
        @param source_drone: Source Drone
        @param destination_drone: Destination Drone
        @param to_send_ts:
        """

        if isinstance(packet_to_send, DataPacket):

            self.metric_class.all_data_packets_in_simulation += 1

        else:

            self.metric_class.all_control_packets_in_simulation += 1

        self.packets.append((packet_to_send, source_drone, destination_drone, to_send_ts))

    def run_medium(self, current_ts):
        """

        @param current_ts:
        @return:
        """

        indices_to_drop = []
        self_packets_deep_copy = self.packets[:]

        self.packets = []

        for index, packet in enumerate(self_packets_deep_copy):

            packet_to_send, source_drone, destination_drone, to_send_ts = packet

            # time to send this packet_to_send
            if to_send_ts == current_ts:

                indices_to_drop.append(index)

                if source_drone.identifier != destination_drone.identifier:

                    drones_distance = util.euclidean_distance(source_drone.coordinates, destination_drone.coordinates)

                    if drones_distance <= min(source_drone.communication_range, destination_drone.communication_range):

                        if destination_drone.routing_algorithm.channel_success(drones_distance):

                            # destination drone receives packet_to_send
                            destination_drone.routing_algorithm.drone_reception(source_drone, packet_to_send)

        self_packets_deep_copy = [self_packets_deep_copy[i] for i in range(len(self_packets_deep_copy)) if i not in indices_to_drop]

        self.packets += self_packets_deep_copy
