from src.routing_algorithms.georouting import GeoRouting
from src.routing_algorithms.random_routing import RandomRouting
from src.routing_algorithms.no_routing import NoRouting
from src.mac_protocol.depot_mac_rnd import RandomDepotMAC
from src.mac_protocol.depot_mac_round_robin import RoundRobinDepotMAC
from src.mac_protocol.ai_depot_mac import BanditDepotMAC
from enum import Enum

"""
This file contains all the constants and parameters of the simulator.
It comes handy when you want to make one shot simulations, making parameters and constants vary in every
simulation. For an extensive experimental campaign read the header at src.simulator.
Attributes that one needs tweak often are tagged with # ***
"""


# Do not modify this class
class MACAlgorithm(Enum):
    RND = RandomDepotMAC
    RR = RoundRobinDepotMAC
    AI = BanditDepotMAC

    @staticmethod
    def keylist():
        return list(map(lambda c: c.name, MACAlgorithm))
# Do not modify this class

# Parameters that you can change

# Change MAC_ALGORITHM to choose the mac protocol you want to use:
#     RND = RandomDepotMAC
#     RR = RoundRobinDepotMAC
#     AI = BanditDepotMAC (your protocol)
MAC_ALGORITHM = MACAlgorithm.RND

# print for each delivered message the feedback if you need to debug
MAC_PRINT_STATS = False

# Plot the distribution of received packets on the terminal, True here requires pip install matplotlib-terminal
# It works only on PyCharm Terminal
PLOT_HISTOGRAMS = True
MATPLOTLIB_TERMINAL = True

# Show the simulation if True
PLOT_SIM = False


######################DO NOT MODIFY ANYTHING ELSE BELOW################################


# ----------------------- PATH DRONES -----------------------------------------#
CIRCLE_PATH = True  # bool: whether to use cirlce paths around the depot
DEMO_PATH = False  # bool: whether to use handcrafted tours or not
# to set up handcrafted torus see utilities.utilities
PATH_FROM_JSON = False  # bool: whether to use the path (for drones) store in the JSONS_PATH_PREFIX,
# otherwise path are generated online
JSONS_PATH_PREFIX = "data/tours/RANDOM_missions{}.json"  # str: the path to the drones tours,
# the {} should be used to specify the seed -> es. data/tours/RANDOM_missions1.json for seed 1.
RANDOM_STEPS = [250, 500, 700, 900, 1100,
                1400]  # the step after each new random directions is taken, in case of dynamic generation
RANDOM_START_POINT = True  # bool whether the drones start the mission at random positions

# ------------------------------- CONSTANTS ------------------------------- #

DEBUG = False  # bool: whether to print debug strings or not.
EXPERIMENTS_DIR = "data/experiments/"  # output data : the results of the simulation

# drawaing
WAIT_SIM_STEP = 0  # float: seconds, pauses the rendering for 'DELAY_PLOT' seconds.
SKIP_SIM_STEP = 5  # int: steps, plot the simulation every 'RENDERING_STEP' steps. At least 1.
DRAW_SIZE = 700  # int: size of the drawing window.
IS_SHOW_NEXT_TARGET_VEC = True  # bool : whether show the direction and next target of the drone

SAVE_PLOT = False  # bool: whether to save the plots of the simulation or not.
SAVE_PLOT_DIR = "data/plots/"

# add constants here...

# ----------------------------- SIMULATION PARAMS. ---------------------------- #
SIM_DURATION = 15000  # int: steps of simulation. # ***
TS_DURATION = 0.150  # float: seconds duration of a step in seconds.
SEED = 20  # int: seed of this simulation.

N_DRONES = 10  # int: number of drones. # ***
ENV_WIDTH = 1500  # float: meters, width of environment.
ENV_HEIGHT = 1500  # float: meters, height of environment.

# events
EVENTS_DURATION = 1  # SIM_DURATION  # int: steps, number of time steps that an event lasts  -> to seconds = step * step_duration.
D_FEEL_EVENT = 1  # int: steps, a new packet is felt (generated on the drone) every 'D_FEEL_EVENT' steps. # ***
P_FEEL_EVENT = .8  # float: probability that the drones feels the event generated on the drone. # ***

""" e.g. given D_FEEL_EVENT = 500, P_FEEL_EVENT = .5, every 500 steps with probability .5 the drone will feel an event."""

# drones
COMMUNICATION_RANGE_DRONE = 200  # float: meters, communication range of the drones.
SENSING_RANGE_DRONE = 0  # float: meters, the sensing range of the drones.
DRONE_SPEED = 8  # float: m/s, drone speed.
DRONE_MAX_BUFFER_SIZE = 100  # int: max number of packets in the buffer of a drone.
DRONE_MAX_ENERGY = 1000000  # int: max energy of a drone.

# depot
DEPOT_COMMUNICATION_RANGE = 200  # float: meters, communication range of the depot.
DEPOT_COO = (750, 750)  # (float, float): coordinates of the depot.


# ------------------------------- ROUTING PARAMS. ------------------------------- #
class RoutingAlgorithm(Enum):
    GEO = GeoRouting
    RND = RandomRouting
    NO = NoRouting

    @staticmethod
    def keylist():
        return list(map(lambda c: c.name, RoutingAlgorithm))


class ChannelError(Enum):
    UNIFORM = 1
    GAUSSIAN = 2
    NO_ERROR = 3

    @staticmethod
    def keylist():
        return list(map(lambda c: c.name, ChannelError))


ROUTING_ALGORITHM = RoutingAlgorithm.NO  # no routing in this scenario!!
CHANNEL_ERROR_TYPE = ChannelError.GAUSSIAN


# ------------------------------- MAC PARAMS. ------------------------------- #


class GenerationPattern(Enum):
    GAUSSIAN = 0
    UNIFORM = 1
    FIXED_PROB = 2


RANDOM_GENERATION_PATTERN = GenerationPattern.GAUSSIAN

COMMUNICATION_P_SUCCESS = 1  # float: probability to have success in a communication.
GUASSIAN_SCALE = .9  # float [0,1]: scale the error probability of the guassian -> success * GUASSIAN_SCALER
PACKETS_MAX_TTL = 200  # float: threshold in the maximum number of hops. Causes loss of packets.
RETRANSMISSION_DELAY = 10  # int: how many time steps to wait before transmit again (for k retransmissions). # ---  #delta_k

# ------------------------------------------- ROUTING MISC --------------------------------- #
HELLO_DELAY = 5  # int : how many time steps wait before transmit again an hello message
RECEPTION_GRANTED = 0.95  # float : the min amount of success to evalute a neigh as relay
LIL_DELTA = 1  # INT:  > 0
OLD_HELLO_PACKET = 50

ROOT_EVALUATION_DATA = "data/evaluation_tests/"

NN_MODEL_PATH = "data/nnmodels/"

# --------------- new cell probabilities -------------- #
CELL_PROB_SIZE_R = 1.875  # the percentage of cell size with respect to drone com range
ENABLE_PROBABILITIES = False
