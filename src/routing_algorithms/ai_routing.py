import random

import numpy as np
import src.utilities.config
from src.utilities import utilities as util
from src.routing_algorithms.BASE_routing import BASE_routing
from enum import Enum, auto
from matplotlib import pyplot as plt

class Action(Enum):
    KEEP = auto()
    GIVE_FERRY = auto()
    GIVE_NODE = auto()


class AIRouting(BASE_routing):
    def __init__(self, drone, simulator):
        BASE_routing.__init__(self, drone, simulator)
        # random generator
        self.rnd_for_routing_ai = np.random.RandomState(self.simulator.seed)
        self.taken_actions = {}  #id event : (old_action)

        num_of_drones = simulator.n_drones
        self.num_of_ferries = AIRouting.__get_number_of_ferries()

        initial_action_values = np.zeros(num_of_drones)
        # Higher values if a drone is a ferry.
        initial_action_values[:self.num_of_ferries] = 1
        # Q table
        self.Q_table = initial_action_values

        # number of actions taken by each drone
        self.action_drones = np.zeros(num_of_drones)
        # total reward for each drone
        self.total_reward = np.zeros(num_of_drones)

    @staticmethod
    def __get_number_of_ferries():
        return src.utilities.config.FERRY


    def feedback(self, drone, id_event, delay, outcome):
        """ return a possible feedback, if the destination drone has received the packet """
        # Packets that we delivered and still need a feedback
        if bool(self.taken_actions):
            print("----------------", id_event, "----------------")
            print(self.drone.identifier, "----------", self.taken_actions)
            print(self.drone.identifier, "----------", drone, id_event, delay, outcome)
        # outcome == -1 if the packet/event expired; 0 if the packets has been delivered to the depot
        # Feedback from a delivered or expired packet


        # negative reward = -1 <-- hyperpameter tuning
        # positive if 0 then 1, otherwise 1/delay
        # here goes the formula

        # Be aware, due to network errors we can give the same event to multiple drones and receive multiple feedback for the same packet!!
        # NOTE: reward or update using the old action!!
        # STORE WHICH ACTION DID YOU TAKE IN THE PAST.
        # do something or train the model (?)
        history = {}
        if id_event in self.taken_actions:
            action = self.taken_actions[id_event]

            del self.taken_actions[id_event]

    def relay_selection(self, opt_neighbors, pkd):
        """ arg min score  -> geographical approach, take the drone closest to the depot """

        # Only if you need --> several features:
        # cell_index = util.TraversedCells.coord_to_cell(size_cell=self.simulator.prob_size_cell,
        #                                                width_area=self.simulator.env_width,
        #                                                x_pos=self.drone.coords[0],  # e.g. 1500
        #                                                y_pos=self.drone.coords[1])[0]  # e.g. 500
        # print(cell_index)

        best_drone_distance_from_depot = util.euclidean_distance(self.simulator.depot.coords, self.drone.coords)
        best_drone = None

        for hpk, drone_instance in opt_neighbors:

            exp_position = self.__estimated_neighbor_drone_position(hpk)
            exp_distance = util.euclidean_distance(exp_position, self.simulator.depot.coords)
            if exp_distance < best_drone_distance_from_depot:
                best_drone_distance_from_depot = exp_distance
                best_drone = drone_instance

                if drone_instance.identifier < self.num_of_ferries:
                    self.__update_actions(pkd.event_ref.identifier, drone_instance, Action.GIVE_FERRY, self.simulator.cur_step)

                else:
                    self.__update_actions(pkd.event_ref.identifier, drone_instance, Action.GIVE_NODE, self.simulator.cur_step)

            else:
                self.__update_actions(pkd.event_ref.identifier, drone_instance, Action.KEEP, self.simulator.cur_step)

        # self.drone.history_path (which waypoint I traversed. We assume the mission is repeated)
        # self.drone.residual_energy (that tells us when I'll come back to the depot).
        #  .....

        # AT THE BEGINNING choose the one nearest (advanced geo)
        # at some point (?) <-- ????????? after we received 10 feedbacks ????
        # with probability 1-e we choose the one with the highest Q
        # with probability (2/3)e we choose the nearest
        # with probability (1/3)e we choose random

        # Store your current action --- you can add several stuff if needed to take a reward later

        return best_drone  # here you should return a drone object!

    def print(self):
        """
            This method is called at the end of the simulation, can be useful to print some
                metrics about the learning process
        """
        pass

    #Private methods
    def __update_actions(self, pkd_id, neighbor, type_action, timestamp):
        if pkd_id in self.taken_actions:
            value = self.taken_actions.get(pkd_id)
            value.append(tuple((neighbor, type_action, timestamp)))
            self.taken_actions[pkd_id] = value

        else:
            self.taken_actions[pkd_id] = list(tuple((neighbor, type_action, timestamp)))

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

