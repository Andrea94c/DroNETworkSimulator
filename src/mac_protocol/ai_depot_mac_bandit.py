
from src.entities.uav_entities import Drone
from src.mac_protocol.depot_mac import DepotMAC

from src.utilities import config
import numpy as np
import matplotlib.pyplot as plt

"""
The class is responsable to allocate communication resources to neighbors drones that want to offload data to the depot.
We work over an semplified TDMA approach, each time step only one drone can receive the resource and communicate a packet to the depot. 
"""
class AIDepotMAC(DepotMAC):
    '''
    epsilon-greedy k-bandit problem

    Inputs
    =====================================================
    k: number of arms (drones) (int)
    eps: probability of random action 0 < eps < 1 (float)
    iters: number of steps (int)
    mu: set the average rewards for each of the k-arms (drones).
        Set to "random" for the rewards to be selected from
        a normal distribution with mean = 0.
        Set to "sequence" for the means to be ordered from
        0 to k-1.
        Pass a list or array of length = k for user-defined
        values.
    '''

    def __init__(self, simulator, depot, eps=0.05, mu="uniform"):
        DepotMAC.__init__(self, simulator, depot)
        # random generator
        self.rnd_for_mac = np.random.RandomState(self.simulator.seed)
        # Number of arms (drones)
        self.k = self.simulator.n_drones
        # Search probability
        self.eps = eps
        # Step count
        self.n = 0
        # Step count for each arm
        self.k_n = np.zeros(self.k)
        # Total mean reward
        self.mean_reward = 0
        # Mean reward for each arm
        self.k_reward = np.zeros(self.k)
        # mean rewards in the time
        self.rewards = []
        # last queries drone
        self.additional_penalty = np.zeros(self.k)

    def action(self):
        # increase steps
        self.n += 1
        # Generate random number
        p = self.rnd_for_mac.rand()
        if self.n == 0:
            a = self.rnd_for_mac.choice(self.k)
        elif p < self.eps:
            # Randomly select an action
            a = self.rnd_for_mac.choice(self.k)
        else:
            # Take greedy action
            a = np.argmax([self.k_reward[i] + self.additional_penalty[i] for i in range(self.k)])

        return a

    def allocate_resource_to_drone(self, drones: list, cur_step: int) -> Drone:
        """ Return the drone to who allocate bandwith for upload data in this step """
        if cur_step == config.SIM_DURATION - 1:
            self.print()

        # implement here your intelligent logic
        if self.last_feedback is not None:  # we effectively have a feedback
            chosen_drone = self.last_feedback[0]
            transmission = self.last_feedback[1]
            missing_packets = self.last_feedback[2]

            reward = np.nan
            if missing_packets > 0:
                reward = 500
            elif not transmission and missing_packets == 0:
                reward = -5
            elif transmission and missing_packets == 0:
                reward = 5
            else:
                print("ERROR this case was not expected")
                exit(1)

            self.update_weights(chosen_drone.identifier, reward)

        next_action = self.action()
        self.add_penalty(next_action)
        return drones[next_action]

    def add_penalty(self, action):
        """ add regret """
        self.additional_penalty[action] = 0
        for ac_ in range(self.k):
            if ac_ == action:
                continue
            self.additional_penalty[ac_] += 0.2

    def update_weights(self, action, reward):
        """ update the weights """
        self.k_n[action] += 1

        # Update results for a_k
        self.k_reward[action] = self.k_reward[action] + (reward - self.k_reward[action]) / self.k_n[action]

        # Update total
        self.mean_reward = self.mean_reward + (reward - self.mean_reward) / self.n

        self.rewards.append(self.mean_reward)

    def print(self):
        print(self.k_reward)
        plt.figure(figsize=(12, 8))
        plt.plot(self.rewards, label="$\epsilon=0.1$")
        plt.legend(bbox_to_anchor=(1.3, 0.5))
        plt.xlabel("Iterations")
        plt.ylabel("Average Reward")
        plt.title("Average reward")
        plt.show()
