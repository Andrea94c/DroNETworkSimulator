

from src.entities.uav_entities import DataPacket, ACKPacket, HelloPacket, Packet
from src.utilities import utilities as util
from src.utilities import config

from scipy.stats import norm
import abc

class BASE_routing(metaclass=abc.ABCMeta):

    def __init__(self, drone, simulator):
        """ The drone that is doing routing and simulator object. """

        self.drone = drone
        self.simulator = simulator

        if self.simulator.communication_error_type == config.ChannelError.GAUSSIAN:
            self.buckets_probability = self.__init_guassian()

        self.current_n_transmission = 0
        self.hello_messages = {}  #{ drone_id : most recent hello packet}
        self.network_disp = simulator.network_dispatcher
        self.simulator = simulator
        self.no_transmission = False

    @abc.abstractmethod
    def relay_selection(self, geo_neighbors, pkd):
        pass

    def routing_close(self, drones, cur_step):
        self.no_transmission = False

    def drone_reception(self, src_drone, packet: Packet, current_ts):
        """ handle reception an ACKs for a packets """
        if isinstance(packet, HelloPacket):
            src_id = packet.src_drone.identifier
            self.hello_messages[src_id] = packet  # add packet to our dictionary

        elif isinstance(packet, DataPacket):
            self.no_transmission = True
            self.drone.accept_packets([packet])
            # build ack for the reception
            ack_packet = ACKPacket(self.drone, src_drone, self.simulator, packet, current_ts)
            self.unicast_message(ack_packet, self.drone, src_drone, current_ts)

        elif isinstance(packet, ACKPacket):
            self.drone.remove_packets([packet.acked_packet])
            # packet.acked_packet.optional_data
            # print(self.is_packet_received_drone_reward, "ACK", self.drone.identifier)
            if self.drone.buffer_length() == 0:
                self.current_n_transmission = 0
                self.drone.move_routing = False

    def drone_identification(self, drones, cur_step):
        """ handle drone hello messages to identify neighbors """
        # if self.drone in drones: drones.remove(self.drone)  # do not send hello to yourself
        if cur_step % config.HELLO_DELAY != 0:  # still not time to communicate
            return

        my_hello = HelloPacket(self.drone, cur_step, self.simulator, self.drone.coords,
                               self.drone.speed,
                               self.drone.move_routing and not self.drone.come_back_to_mission,
                               self.drone.buffer_length(),
                               self.drone.next_target())

        self.broadcast_message(my_hello, self.drone, drones, cur_step)

    def routing(self, depot, drones, cur_step):
        # set up this routing pass
        self.drone_identification(drones, cur_step)

        self.send_packets(cur_step)

        # close this routing pass
        self.routing_close(drones, cur_step)


    def send_packets(self, cur_step):
        """ procedure 3 -> choice next hop and try to send it the data packet """

        # FLOW 0
        if self.no_transmission or self.drone.buffer_length() == 0:
            return

        # FLOW 1
        for coords in self.simulator.depot.list_of_coords:
            if util.euclidean_distance(coords, self.drone.coords) <= self.simulator.depot.communication_range:
                # add error in case
                depot_index = -1 if coords == self.simulator.depot.list_of_coords[0] else -2
                self.transfer_to_depot(self.drone.depot, cur_step, depot_index)

                self.drone.move_routing = False
                self.current_n_transmission = 0
                return

        # only drone 0 will send packets
        #if self.drone.identifier < config.FERRY:
        #   return

        # TODO: Aspetta che lo faccia a ogni drone_retransmission_delta
        if cur_step % self.simulator.drone_retransmission_delta == 0:

            opt_neighbors = []
            for hpk_id in self.hello_messages:
                hpk: HelloPacket = self.hello_messages[hpk_id]

                # check if packet is too old
                if hpk.time_step_creation < cur_step - config.OLD_HELLO_PACKET:
                    continue

                opt_neighbors.append((hpk, hpk.src_drone))

            # send packets
            for pkd in self.drone.all_packets():
                self.simulator.metrics.mean_numbers_of_possible_relays.append(len(opt_neighbors))
                best_neighbor = self.relay_selection(opt_neighbors, pkd)  # compute score
                if best_neighbor == -1 or best_neighbor == -2:
                    if not self.drone.move_routing:
                        self.drone.return_coords = self.simulator.depot.list_of_coords[(best_neighbor*-1)-1]
                        # add penalty to change trajectories
                        self.simulator.metrics.energy_spent_for_active_movement[self.drone.identifier] += 25
                    self.drone.move_routing = True
                elif best_neighbor is not None:
                    self.unicast_message(pkd, self.drone, best_neighbor, cur_step)

                self.current_n_transmission += 1

    def geo_neighborhood(self, drones, no_error=False):
        """ returns the list all the Drones that are in self.drone neighbourhood (no matter the distance to depot),
            in all direction in its transmission range, paired with their distance from self.drone """

        closest_drones = []  # list of this drone's neighbours and their distance from self.drone: (drone, distance)

        for other_drone in drones:
            if self.drone.identifier != other_drone.identifier:                                   # not the same drone
                drones_distance = util.euclidean_distance(self.drone.coords, other_drone.coords)  # distance between two drones

                if drones_distance <= min(self.drone.communication_range, other_drone.communication_range):  # one feels the other & vv

                    # CHANNEL UNPREDICTABILITY
                    if self.channel_success(drones_distance, no_error=no_error):
                        closest_drones.append((other_drone, drones_distance))

        return closest_drones

    def channel_success(self, drones_distance, no_error=False):
        """ Precondition: two drones are close enough to communicate. Return true if the communication
        goes through, false otherwise.  """

        assert(drones_distance <= self.drone.communication_range)

        if no_error:
            return True

        if self.simulator.communication_error_type == config.ChannelError.NO_ERROR:
            return True

        elif self.simulator.communication_error_type == config.ChannelError.ON_DEVICE:
            return self.__on_device_error(drones_distance)

        elif self.simulator.communication_error_type == config.ChannelError.UNIFORM:
            return self.simulator.rnd_routing.rand() <= self.simulator.drone_communication_success

        elif self.simulator.communication_error_type == config.ChannelError.GAUSSIAN:
            return self.simulator.rnd_routing.rand() <= self.gaussian_success_handler(drones_distance)

    def broadcast_message(self, packet, src_drone, dst_drones, curr_step):
        """ send a message to my neigh drones"""
        for d_drone in dst_drones:
            self.unicast_message(packet, src_drone, d_drone, curr_step)

    def unicast_message(self, packet, src_drone, dst_drone, curr_step):
        """ send a message to my neigh drones"""
        # Broadcast using Network dispatcher
        self.simulator.network_dispatcher.send_packet_to_medium(packet, src_drone, dst_drone, curr_step + config.LIL_DELTA)

    def gaussian_success_handler(self, drones_distance):
        """ get the probability of the drone bucket """
        bucket_id = int(drones_distance / self.radius_corona) * self.radius_corona
        return self.buckets_probability[bucket_id] * config.GUASSIAN_SCALE

    def transfer_to_depot(self, depot, cur_step, depot_index):
        """ self.drone is close enough to depot and offloads its buffer to it, restarting the monitoring
                mission from where it left it
        """
        depot.transfer_notified_packets(self.drone, cur_step, depot_index)
        self.drone.empty_buffer()
        self.drone.move_routing = False

    # --- PRIVATE ---
    def __on_device_error(self, drones_distance):
        # we approximate 3 level of distance and error
        if drones_distance <= self.drone.communication_range / 3:
            return self.simulator.rnd_routing.rand() <= self.drone.channel_success_rate
        elif drones_distance <= self.drone.communication_range * 2 / 3:
            return self.simulator.rnd_routing.rand() <= self.drone.channel_success_rate * (2 / 3)
        else:
            return self.simulator.rnd_routing.rand() <= self.drone.channel_success_rate / 3


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


