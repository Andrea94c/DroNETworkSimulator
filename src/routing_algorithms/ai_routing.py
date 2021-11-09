import random

import numpy as np
import src.utilities.config
from src.utilities import utilities as util
from src.routing_algorithms.BASE_routing import BASE_routing
from enum import Enum, auto


class Action(Enum):
    KEEP = auto()
    GIVE_FERRY = auto()
    GIVE_NODE = auto()


class AIRouting(BASE_routing):
    def __init__(self, drone, simulator):
        BASE_routing.__init__(self, drone, simulator)
        # random generator
        self.rnd_for_routing_ai = np.random.RandomState(self.simulator.seed)
        self.taken_actions = {}  # id event : (old_action)

        num_of_drones = simulator.n_drones
        self.num_of_ferries = AIRouting.__get_number_of_ferries()

        self.epsilon = 0.3
        np.random.seed(self.drone.identifier)

        initial_action_values = np.zeros(num_of_drones)
        # Higher values if a drone is a ferry.
        initial_action_values[:self.num_of_ferries] = 1
        initial_action_values[self.num_of_ferries:] = 0.5
        # Q table
        self.Q_table = initial_action_values

        # number of actions taken by each drone
        self.action_drones = np.zeros(num_of_drones)
        # total reward for each drone (??? I didn't understand this)
        self.total_reward = np.zeros(num_of_drones)

    @staticmethod
    def __get_number_of_ferries():
        return src.utilities.config.FERRY

    def feedback(self, drone, id_event, delay, outcome):
        """ return a possible feedback, if the destination drone has received the packet """
        # Packets that we delivered and still need a feedback
        # if id_event in self.taken_actions:
        #     print("----------------", id_event, "----------------")
        #     print(self.drone.identifier, "----------", self.taken_actions)
        #     print(self.drone.identifier, "----------", drone, id_event, delay, outcome)
        # # outcome == -1 if the packet/event expired; 0 if the packets has been delivered to the depot
        # Feedback from a delivered or expired packet

        # Be aware, due to network errors we can give the same event to multiple drones and receive multiple feedback for the same packet!!
        # NOTE: reward or update using the old action!!
        # STORE WHICH ACTION DID YOU TAKE IN THE PAST.
        # do something or train the model (?)

        neg_rew = [-1, -0.8, -0.6, -0.4, -0.2]
        reward_per_action = {}

        if id_event in self.taken_actions:
            action = self.taken_actions[id_event]
            # action = list(dict.fromkeys(action))
            if outcome == -1:
                i = min(5, len(action))
                reward_per_action = dict(zip(action[-i:][::-1], neg_rew[:i]))
                reward_per_action.update(dict.fromkeys(action[0:-i], 0.1))
                # print(self.drone.identifier, len(action), "ACTION\n", action, "\nREWARD\n", reward_per_action)
            else:
                # print(self.drone.identifier, "-->", action)
                reward_per_action = dict.fromkeys(action, 1/(delay/1000))
            
            # update Q_table according to reward_per_action

            del self.taken_actions[id_event]

    def relay_selection(self, opt_neighbors, pkd):
        """ arg min score  -> geographical approach, take the drone closest to the depot """

        # Only if you need --> several features:
        # cell_index = util.TraversedCells.coord_to_cell(size_cell=self.simulator.prob_size_cell,
        #                                                width_area=self.simulator.env_width,
        #                                                 x_pos=self.drone.coords[0],  # e.g. 1500
        #                                                 y_pos=self.drone.coords[1])[0]  # e.g. 500
        # print(cell_index)

        best_drone = None
        prob = np.random.random()

        # if prob > self.epsilon and self.simulator.cur_step > 300:
        #     # drones that are my neighbours
        #     neighbors = [t[1].identifier for t in opt_neighbors]
        #     # the right choice could be to keep the packet
        #     neighbors.append(self.drone.identifier)
        #     # extracting max of q_values for neighbours
        #     max_ind = np.argmax(self.Q_table[neighbors])
        #     # getting the drone
        #     best_drone = None if max_ind == len(opt_neighbors) else opt_neighbors[max_ind][1]

        #elif prob > (2/3 * self.epsilon) or self.simulator.cur_step < 300:
        best_drone_distance_from_depot = util.euclidean_distance(self.simulator.depot.coords, self.drone.coords, )
        for hpk, drone_instance in opt_neighbors:
            exp_position = self.__estimated_neighbor_drone_position(hpk)
            exp_distance = util.euclidean_distance(exp_position, self.simulator.depot.coords)
            if exp_distance < best_drone_distance_from_depot:
                best_drone_distance_from_depot = exp_distance
                best_drone = drone_instance
        
        #else:
            #best_drone = self.simulator.rnd_routing.choice([v[1] for v in opt_neighbors])

        # self.drone.history_path (which waypoint I traversed. We assume the mission is repeated)
        # self.drone.residual_energy (that tells us when I'll come back to the depot).
        #  .....

        # Store your current action --- you can add several stuff if needed to take a reward later

        action = ""

        # check when we need to add the waypoint_history to the action
        ferry = self.__set_ferry_flag(best_drone)

        # add all the action information collected so far
        if best_drone is None:
            self.__update_actions(pkd.event_ref.identifier, best_drone, Action.KEEP, ferry, self.simulator.cur_step)
            action = Action.KEEP

        elif best_drone.identifier < self.num_of_ferries:
            self.__update_actions(pkd.event_ref.identifier, best_drone, Action.GIVE_FERRY, ferry, self.simulator.cur_step)
            action = Action.GIVE_FERRY

        else:
            self.__update_actions(pkd.event_ref.identifier, best_drone, Action.GIVE_NODE, ferry, self.simulator.cur_step)
            action = Action.GIVE_NODE

        # print(self.drone.identifier, action, pkd.event_ref.identifier, best_drone)

        return best_drone  # here you should return a drone object!

    def print(self):
        """
            This method is called at the end of the simulation, can be useful to print some
                metrics about the learning process
        """
        pass

    # Private methods
    def __update_actions(self, pkd_id, neighbor, type_action, ferry, timestamp):

        drone = neighbor if neighbor is not None else self.drone

        if pkd_id in self.taken_actions:
            value = self.taken_actions.get(pkd_id)
            # if (value[-1])[:2] != tuple((neighbor, type_action)):
            if ferry:
                value.append(tuple((neighbor, type_action, None, timestamp)))
            else:
                region = self.__assign_region(int(drone.coords[0]), int(drone.coords[1]))
                value.append(tuple((neighbor, type_action, region, timestamp)))
            #value.append(tuple((neighbor, type_action)))
            self.taken_actions[pkd_id] = value

        else:
            region = self.__assign_region(int(drone.coords[0]), int(drone.coords[1]))
            self.taken_actions[pkd_id] = [tuple((neighbor, type_action, None, timestamp))] if ferry \
                else [tuple((neighbor, type_action, region, timestamp))]

    def __incremental_estimate_method(self, drone, reward):
        self.action_drones[drone.identifier] += 1
        self.total_reward[drone.identifier] += reward
        self.Q_table[drone.identifier] += ((reward - self.Q_table[drone.identifier])/self.action_drones[drone.identifier])

    def __sample_avg_estimate_method(self, drone, reward):
        self.action_drones[drone.identifier] += 1
        self.total_reward[drone.identifier] += reward
        self.Q_table[drone.identifier] = (self.total_reward[drone.identifier]/self.action_drones[drone.identifier])

    def __greedy(self):
        return self.Q_table.argmax()

    def __epsilon_greedy(self, epsilon):
        p = np.random.random()
        return random.choice(self.Q_table) if p < epsilon else self.__greedy()

    def __is_a_ferry(self, drone):
        return drone.identifier < self.num_of_ferries

    def __assign_region(self, x, y):

        regions_matrix = np.reshape(np.arange(1, 10), (3,3))

        size = self.simulator.env_width
        coords = list(range(0, size))
        splitter = int(size / 3)

        split_1, split_2, split_3 = coords[0:splitter], coords[splitter:splitter*2], coords[splitter*2:]
        coords = (split_1, split_2, split_3)
        index_x = [i for i in range(len(coords))if x in coords[i]][0]
        index_y = [i for i in range(len(coords))if y in coords[i]][0]

        return regions_matrix[index_y][index_x]

    def __set_ferry_flag(self, best_drone):
        ferry = False
        if best_drone is None and self.__is_a_ferry(self.drone):
            ferry = True
        elif self.__is_a_ferry(self.drone) and (best_drone is not None and self.__is_a_ferry(best_drone)):
            ferry = True

        return ferry


    def __estimated_neighbor_drone_position(self, hello_message):
        """ estimate the current position of the drone """

        # get known info about the neighbor drone
        hello_message_time = hello_message.time_step_creation
        known_position = hello_message.cur_pos
        known_speed = hello_message.speed
        known_next_target = hello_message.next_target

        # compute the time elapsed since the message sent and now
        # elapsed_time in seconds = elapsed_time in steps * step_duration_in_seconds
        elapsed_time = (self.simulator.cur_step - hello_message_time) * self.simulator.time_step_duration  # seconds

        # distance traveled by drone
        distance_traveled = elapsed_time * known_speed

        # direction vector
        a, b = np.asarray(known_position), np.asarray(known_next_target)
        v_ = (b - a) / np.linalg.norm(b - a)

        # compute the expect position
        c = a + (distance_traveled * v_)

        return tuple(c)
