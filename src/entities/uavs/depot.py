from src.entities.entity import Entity


class Depot(Entity):
    """
    The depot is an Entity
    """

    def __init__(self, simulator, coordinates: tuple, communication_range:float):
        super().__init__(simulator=simulator, identifier=id(self), coordinates=coordinates)

        self.communication_range = communication_range
        self.__buffer = list()  # also with duplicated packets

    @property
    def all_packets(self):
        """
        Return all packets within the depot buffer
        @return: a list of Packets
        """

        return self.__buffer

    def transfer_notified_packets(self, drone):
        """
        This function is called when a Drone wants to offload packets to the depot
        @param drone:
        @return:
        """

        packets_to_offload = drone.all_packets
        self.__buffer += packets_to_offload

        for packet in packets_to_offload:

            # add metrics: all the packets notified to the depot

            self.simulator.logger.add_drones_packet_to_depot(timestep=self.simulator.cur_step,
                                                             packet=packet,
                                                             source_drone=drone)
            packet.time_delivery = self.simulator.cur_step

