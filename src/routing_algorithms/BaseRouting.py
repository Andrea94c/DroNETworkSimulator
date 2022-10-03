from src.entities.events.event import Event
from src.entities.packets.packets import Packet, HelloPacket, DataPacket, ACKPacket
from src.utilities import utilities as util
from src.utilities import config
from scipy.stats import norm
import abc


class BaseRouting(metaclass=abc.ABCMeta):
    """
    This class is used as baseclass to build new routing protocols.
    """

    def __init__(self, simulator, drone):
        """

        @param drone: The drone that needs to perform routing
        @param simulator: The Simulator instance
        """

        self.drone = drone
        self.simulator = simulator

        if self.simulator.communication_error_type == config.ChannelError.GAUSSIAN:
            self.buckets_probability = self.__init_guassian()

        self.current_n_transmission = 0
        self.hello_messages = {}  # { drone_id : most recent hello packet}
        self.network_disp = simulator.network_dispatcher
        self.no_transmission = False

    @abc.abstractmethod
    def relay_selection(self, neighbors: list):
        """
        Abstract method to implement the routing protocol
        @param neighbors: a list of neighbors drones
        """
        pass

    def routing_close(self):
        self.no_transmission = False

    def drone_reception(self, source_drone, packet: Packet):
        """
        This method handles the drones packet reception.
        @param source_drone: Drone that sent the packet
        @param packet: Packet sent
        @return:
        """

        if isinstance(packet, HelloPacket):

            source_drone_id = packet.source_drone.identifier

            self.hello_messages[source_drone_id] = packet  # add packet to our dictionary

        elif isinstance(packet, DataPacket):

            self.no_transmission = True
            self.drone.accept_packets([packet])

            null_event = Event(self.simulator, (-1, -1), -1)

            ack_packet = ACKPacket(simulator=self.simulator,
                                   source_drone=self.drone,
                                   destination_drone=source_drone,
                                   acked_packet=packet,
                                   event_ref=null_event)

            self.unicast_message(packet_to_send=ack_packet, source_drone=self.drone, destination_drone=source_drone)

        elif isinstance(packet, ACKPacket):

            self.drone.remove_packets([packet.acked_packet])

            if self.drone.buffer_length == 0:

                self.current_n_transmission = 0
                self.drone.move_routing = False

    def drone_identification(self, drones):
        """
        It sends to the neighbours an HelloPacket
        @param drones:
        @return:
        """

        # still not time to communicate
        if self.simulator.cur_step % config.HELLO_DELAY != 0:

            return 0

        null_event = Event(simulator=self.simulator,
                           coordinates=(-1, -1),
                           current_time=-1)

        hello_packet = HelloPacket(simulator=self.simulator,
                                   source_drone=self.drone,
                                   current_position=self.drone.coordinates,
                                   current_speed=self.drone.speed,
                                   next_target=self.drone.next_target(),
                                   event_ref=null_event)

        self.broadcast_message(packet_to_send=hello_packet,
                               source_drone=self.drone,
                               destination_drones=drones)

    def routing(self, drones):
        """

        @param drones:
        @return:
        """

        # set up this routing pass
        self.drone_identification(drones)

        self.send_packets()

        # close this routing pass
        self.routing_close()

    def send_packets(self):
        """

        @return:
        """

        # FLOW 1: if it is not possible to communicate with other Entities or there are no packets to send
        if self.no_transmission or self.drone.buffer_length == 0:

            return 0

        # FLOW 2: the Depot is close enough to offload packets directly
        communication_radius_scaling_factor = 0.98
        if self.drone.distance_from_depot <= (self.drone.communication_range + self.drone.depot.communication_range) * communication_radius_scaling_factor:

            self.transfer_to_depot(self.drone.depot)
            self.drone.move_routing = False
            self.current_n_transmission = 0

            return 0

        # FLOW 3: find the best relay through the routing algorithm
        if self.simulator.cur_step % self.simulator.drone_retransmission_delta == 0:

            opt_neighbors = []

            for hello_packet_id in self.hello_messages:

                hello_packet = self.hello_messages[hello_packet_id]

                # check if packet is too old, if so discard the packet
                if hello_packet.time_step_creation < self.simulator.cur_step - config.OLD_HELLO_PACKET:

                    continue

                opt_neighbors.append((hello_packet, hello_packet.source_drone))

            if opt_neighbors:

                best_neighbor = self.relay_selection(opt_neighbors)  # compute score

                if best_neighbor is not None:

                    # send packets
                    for packet in self.drone.all_packets:

                        self.unicast_message(packet_to_send=packet,
                                             source_drone=self.drone,
                                             destination_drone=best_neighbor)

            self.current_n_transmission += 1

    def geo_neighborhood(self, drones, no_error=False):
        """
        returns the list all the Drones that are in self.drone neighbourhood (no matter the distance to depot),
        in all direction in its transmission range, paired with their distance from self.drone
        """

        closest_drones = []  # list of this drone's neighbours and their distance from self.drone: (drone, distance)

        for other_drone in drones:
            if self.drone.identifier != other_drone.identifier:  # not the same drone
                drones_distance = util.euclidean_distance(self.drone.coordinates,
                                                          other_drone.coordinates)  # distance between two drones

                if drones_distance <= min(self.drone.communication_range,
                                          other_drone.communication_range):  # one feels the other & vv

                    # CHANNEL UNPREDICTABILITY
                    if self.channel_success(drones_distance, no_error=no_error):
                        closest_drones.append((other_drone, drones_distance))

        return closest_drones

    def channel_success(self, drones_distance, no_error=False):
        """
        Precondition: two drones are close enough to communicate. Return true if the communication
        goes through, false otherwise.
        """

        assert drones_distance <= self.drone.communication_range

        if no_error:

            return True

        if self.simulator.communication_error_type == config.ChannelError.NO_ERROR:

            return True

        elif self.simulator.communication_error_type == config.ChannelError.UNIFORM:

            return self.simulator.rnd_routing.rand() <= self.simulator.drone_communication_success

        elif self.simulator.communication_error_type == config.ChannelError.GAUSSIAN:

            return self.simulator.rnd_routing.rand() <= self.gaussian_success_handler(drones_distance)

    def broadcast_message(self, packet_to_send, source_drone, destination_drones):
        """
        It Sends a message to all the neighbours
        @param packet_to_send:
        @param source_drone:
        @param destination_drones:
        @return:
        """

        for drone in destination_drones:

            self.unicast_message(packet_to_send=packet_to_send,
                                 source_drone=source_drone,
                                 destination_drone=drone)

    def unicast_message(self, packet_to_send, source_drone, destination_drone):
        """
        It Sends a message directly to a single drone
        @param packet_to_send:
        @param source_drone:
        @param destination_drone:
        @return:
        """

        self.simulator.network_dispatcher.send_packet_to_medium(packet_to_send=packet_to_send,
                                                                source_drone=source_drone,
                                                                destination_drone=destination_drone,
                                                                to_send_ts=self.simulator.cur_step + config.LIL_DELTA)

    def gaussian_success_handler(self, drones_distance):
        """ get the probability of the drone bucket """
        bucket_id = int(drones_distance / self.radius_corona) * self.radius_corona
        return self.buckets_probability[bucket_id] * config.GAUSSIAN_SCALE

    def transfer_to_depot(self, depot):
        """
        self.drone is close enough to depot and offloads its buffer to it, restarting the monitoring
        mission from where it left it
        """
        depot.transfer_notified_packets(self.drone)
        self.drone.empty_buffer()
        self.drone.move_routing = False

    # --- PRIVATE ---
    def __init_guassian(self, mu=0, sigma_wrt_range=1.15, bucket_width_wrt_range=.5):

        # bucket width is 0.5 times the communication radius by default
        self.radius_corona = int(self.drone.communication_range * bucket_width_wrt_range)

        # sigma is 1.15 times the communication radius by default
        sigma = self.drone.communication_range * sigma_wrt_range

        max_prob = norm.cdf(mu + self.radius_corona, loc=mu, scale=sigma) - norm.cdf(0, loc=mu, scale=sigma)

        # maps a bucket starter to its probability of gaussian success
        buckets_probability = {}
        for bk in range(0, self.drone.communication_range, self.radius_corona):
            prob_leq = norm.cdf(bk, loc=mu, scale=sigma)
            prob_leq_plus = norm.cdf(bk + self.radius_corona, loc=mu, scale=sigma)
            prob = (prob_leq_plus - prob_leq) / max_prob
            buckets_probability[bk] = prob

        return buckets_probability
