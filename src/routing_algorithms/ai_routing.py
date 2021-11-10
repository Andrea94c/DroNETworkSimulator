import numpy as np
import src.utilities.config
from src.utilities import utilities as util
from src.routing_algorithms.BASE_routing import BASE_routing
from enum import Enum, auto

from src.utilities.random_waypoint_generation import next_target

def assign_region(x, y, width):

    regions_matrix = np.reshape(np.arange(1, 10), (3,3))
    splitter = int(width / 3)
    index_x = 0 if x < splitter else (1 if x < splitter * 2 else 2)
    index_y = 0 if y < splitter else (1 if y < splitter * 2 else 2)

    return regions_matrix[index_y][index_x]

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

        self.num_of_ferries = AIRouting.__get_number_of_ferries()

        self.epsilon = 0.1

        # number of times an action has been taken
        self.n_actions = {}
        # rewards for an action up to now
        self.rew_actions = {}
        # Q_table
        self.Q_table = {}

    @staticmethod
    def __get_number_of_ferries():
        return src.utilities.config.FERRY

    #if outcome positive then drone is the drone that delivered the package to the depot
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

        neg_rewards = [-1, -0.8, -0.6, -0.4, -0.2]
        reward_per_action = {}

        if id_event in self.taken_actions:
            action = self.taken_actions[id_event]
            action.reverse()
            if outcome == -1:
                i = min(5, len(action))
                reward_per_action = dict(zip(action[:i], neg_rewards))
                reward_per_action.update(dict.fromkeys(action[i:], 0.1))
            else:
                reward_per_action = dict.fromkeys(action, 1/delay * 1000)
            
            #update Q_table according to reward_per_action

            n_previous = [self.n_actions[a]+1 if a in self.n_actions else 1 for a in action]
            self.n_actions.update(dict(zip(action, n_previous)))

            #r_previous = [self.rew_actions[a]+reward_per_action[a] if a in self.rew_actions else (reward_per_action[a]+1 if a[0]==Action.GIVE_FERRY and outcome == 0 else reward_per_action[a]) for a in action]
            r_previous = [self.rew_actions[a]+reward_per_action[a] if a in self.rew_actions else reward_per_action[a] for a in action]
            self.rew_actions.update(dict(zip(action, r_previous)))

            q = [self.rew_actions[a]/self.n_actions[a] for a in action]
            self.Q_table.update(dict(zip(action, q)))

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
        region = assign_region(self.drone.coords[0], self.drone.coords[1], self.simulator.env_width)
        prob = self.rnd_for_routing_ai.rand()

        waypoint = self.drone.current_waypoint
        used_Q = False

        useAI = True

        #already did at least one round
        if useAI and waypoint != -1 and waypoint < len(self.drone.waypoint_history): #and prob > self.epsilon
            key_actions = [q for q in self.Q_table if q[1] == region and q[2] == waypoint]
            value_actions = [self.Q_table[k] for k in key_actions]
            if value_actions:
                max_ind = np.argmax(value_actions)
                best_action = key_actions[max_ind][0]
                #drone that are my neighbours
                neighbours = [t[1] for t in opt_neighbors]
                if best_action == Action.KEEP:
                    used_Q = True
                elif best_action == Action.GIVE_FERRY:
                    best_drone = next((n for n in neighbours if n.identifier < self.num_of_ferries), None)
                    used_Q = best_drone is not None
                else:
                    best_drone = next((n for n in neighbours if n.identifier > self.num_of_ferries), None)
                    used_Q = best_drone is not None

        if not used_Q:
            best_drone_distance_from_depot = util.euclidean_distance(self.simulator.depot.coords, self.drone.coords)
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

        action = Action.KEEP if best_drone is None else (Action.GIVE_FERRY if best_drone.identifier < self.num_of_ferries else Action.GIVE_NODE)
        self.__update_actions(pkd.event_ref.identifier, action, region, self.drone.current_waypoint)

        return best_drone  # here you should return a drone object!

    def print(self):
        """
            This method is called at the end of the simulation, can be useful to print some
                metrics about the learning process
        """
        pass

    # Private methods
    def __update_actions(self, pkd_id, type_action, region, step):

        if self.test == "" and self.drone.identifier == 0:
            self.test = pkd_id

        if pkd_id in self.taken_actions:
            #extracting previuos actions for the packet
            value = self.taken_actions.get(pkd_id)
            #save new action only if it's different from last one
            if (value[-1]) != tuple((type_action, region, step)):
                value.append(tuple((type_action, region, step)))
                self.taken_actions[pkd_id] = value

        else:
            self.taken_actions[pkd_id] = [tuple((type_action, region, step))]

    def __is_a_ferry(self, drone):
        return drone.identifier < self.num_of_ferries

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
