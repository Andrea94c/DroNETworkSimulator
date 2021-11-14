import numpy as np
import src.utilities.config
from src.utilities import utilities as util
from src.routing_algorithms.BASE_routing import BASE_routing
from enum import Enum, auto

from src.utilities.random_waypoint_generation import next_target

def assign_region(x, y, width):

    regions_matrix = np.reshape(np.arange(1, 17), (4,4))
    splitter = int(width / 4)
    index_x = 0 if x < splitter else (1 if x < splitter * 2 else (2 if x < splitter * 3 else 3))
    index_y = 0 if y < splitter else (1 if y < splitter * 2 else (2 if y < splitter * 3 else 3))

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
        self.is_ferry = self.drone.identifier < self.num_of_ferries

        self.epsilon = 0.1

        # number of times an action has been taken
        self.n_actions = {}
        # Q_table
        self.Q_table = {}

    @staticmethod
    def __get_number_of_ferries():
        return src.utilities.config.FERRY

    #if outcome positive then drone is the drone that delivered the package to the depot
    def feedback(self, drone, id_event, delay, outcome):
        """ return a possible feedback, if the destination drone has received the packet """
        # outcome == -1 if the packet/event expired; 0 if the packets has been delivered to the depot
        # Feedback from a delivered or expired packet

        # Be aware, due to network errors we can give the same event to multiple drones and receive multiple feedback for the same packet!!

        #neg_rewards = [-2, -1.5, -1, -0.5, -0.3]
        reward_per_action = {}

        if id_event in self.taken_actions:
            action = self.taken_actions[id_event]
            action.reverse() #here actions taken for the packet are from the most recent to the last
            if outcome == -1:
                #i = min(5, len(action))
                #reward_per_action = dict(zip(action[:i], neg_rewards))
                #reward_per_action.update(dict.fromkeys(action[i:], 0.1))
                reward_per_action = dict.fromkeys(action, -2)
            else:
                reward_per_action = dict.fromkeys(action, 1/delay * 1000)
            
            #update Q_table according to reward_per_action

            n_previous = [self.n_actions[a]+1 if a in self.n_actions else 1 for a in action]
            self.n_actions.update(dict(zip(action, n_previous)))

            q = [self.Q_table[a] + 1/self.n_actions[a]*(reward_per_action[a]-self.Q_table[a]) if a in self.Q_table else reward_per_action[a] for a in action]

            self.Q_table.update(dict(zip(action, q)))

            del self.taken_actions[id_event]

    def relay_selection(self, opt_neighbors, pkd):

        best_drone = None

        #current region
        region = assign_region(self.drone.coords[0], self.drone.coords[1], self.simulator.env_width)
        #current step in our mission
        waypoint = self.drone.current_waypoint
        #energy left --> if I have low energy then I should keep the packets, I'm returning to the depot
        energy = self.drone.residual_energy > 1500

        #drone that are my neighbours
        neighbours = [t[1] for t in opt_neighbors]
        #I could be the best choice (keep the packet)
        neighbours.append(None)

        #current Q_table values for possible actions in the current region and for the current waypoint
        key_actions = [q for q in self.Q_table if q[1] == region and q[2] == waypoint and q[0] in neighbours]
        value_actions = [self.Q_table[k] for k in key_actions]
        
        #optimistical initial values
        for n in neighbours:
            t = tuple((n, region, waypoint))
            if t not in key_actions:
                key_actions.append(t)
                #at the beginning we are going to assume that give packets to the ferries is the right choice 
                value = 2 if (n is None and self.is_ferry) or (n is not None and n.identifier < self.num_of_ferries) else 1
                value_actions.append(value)
                self.Q_table[t] = value

        '''if not value_actions:
            #possible_actions = [tuple((type_act, region, waypoint)) for type_act in Action]
            possible_actions = [tuple((n, region, waypoint)) for n in neighbours]
            initial_values = [2 if (n is None and self.drone.identifier < self.num_of_ferries) or n.identifier < self.num_of_ferries else 1 for n in neighbours]
            #initial_values = [2 if a[0]==Action.GIVE_FERRY or (a[0]==Action.KEEP and self.drone.identifier < self.num_of_ferries) else 1 for a in possible_actions]
            self.Q_table.update(dict(zip(possible_actions, initial_values)))
            key_actions = possible_actions
            value_actions = initial_values'''

        #if I'm a ferry I'm going to keep the packets most of the time
        ferry = False
        if self.is_ferry:
            prob = self.rnd_for_routing_ai.rand()
            if prob > 0.01:
                ferry = True

        #already did at least one round
        if waypoint < len(self.drone.waypoint_history) and energy and not ferry:
            #epsilon greedy, with low probability we choose a random action
            prob = self.rnd_for_routing_ai.rand()
            if prob < self.epsilon:
                best_drone = self.rnd_for_routing_ai.choice(neighbours)
            #with high probability we choose what we think is the best action
            else:
                max_ind = np.argmax(value_actions)
                #best_action = key_actions[max_ind][0]
                best_drone = key_actions[max_ind][0]
                '''#if best_action is Keep then best_drone is already None
                if best_action == Action.GIVE_FERRY:
                    best_drone = next((n for n in neighbours if n is not None and n.identifier < self.num_of_ferries), None)
                elif best_action == Action.GIVE_NODE:
                    best_drone = next((n for n in neighbours if n is not None and n.identifier > self.num_of_ferries), None)
                #if the best action is not possible this time, we choose the second best
                if best_drone is None and best_action != Action.KEEP:
                    key_actions = [q for q in key_actions if q[0] != best_action]
                    value_actions = [self.Q_table[k] for k in key_actions]
                    max_ind = np.argmax(value_actions)
                    best_action = key_actions[max_ind][0]
                    best_drone = None if best_action == Action.KEEP else neighbours[0]'''

        #for the first round we choose the best drone according to the estimated position
        elif energy and not ferry:
            best_drone_distance_from_depot = util.euclidean_distance(self.simulator.depot.coords, self.drone.coords)
            for hpk, drone_instance in opt_neighbors:
                exp_position = self.__estimated_neighbor_drone_position(hpk)
                exp_distance = util.euclidean_distance(exp_position, self.simulator.depot.coords)
                if exp_distance < best_drone_distance_from_depot:
                    best_drone_distance_from_depot = exp_distance
                    best_drone = drone_instance

        #store the current action. An action is identified by the choice of what to do with the packet, by the region and by the waypoint
        #action = Action.KEEP if best_drone is None else (Action.GIVE_FERRY if best_drone.identifier < self.num_of_ferries else Action.GIVE_NODE)
        self.__update_actions(pkd.event_ref.identifier, best_drone, region, waypoint)

        return best_drone

    def print(self):
        """
            This method is called at the end of the simulation, can be useful to print some
                metrics about the learning process
        """
        pass

    # Private methods
    def __update_actions(self, pkd_id, action, region, step):

        if pkd_id in self.taken_actions:
            #extracting previuos actions for the packet
            value = self.taken_actions.get(pkd_id)
            #save new action only if it's different from last one
            if (value[-1]) != tuple((action, region, step)):
                value.append(tuple((action, region, step)))
                self.taken_actions[pkd_id] = value

        else:
            #this is the first action for the packet
            self.taken_actions[pkd_id] = [tuple((action, region, step))]

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
