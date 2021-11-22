
""" To clean. """

from src.utilities import config

import pathlib
import time
import json
import matplotlib.pyplot as plt
import pandas as pd
import numpy as np
import pickle
from ast import literal_eval as make_tuple
from src.utilities import random_waypoint_generation
from scipy.special import softmax

class DroneCapabilities():

    def __init__(self, simulator):
        """ The class craete personalized features for each drone """
        self.simulator = simulator
        self.device_rnd = np.random.RandomState(self.simulator.seed + 1)
        self.network_suc_rate = {}   # drone_id : success_rate
        self.speed = {}  # drone_id : speed of the drone
        self.__compute()

    def __compute(self):
        for i in range(self.simulator.n_drones):
            self.speed[i] = self.device_rnd.randint(self.simulator.drone_speed - int(self.simulator.drone_speed) / 2,
                                                    self.simulator.drone_speed + int(self.simulator.drone_speed / 4))
            self.network_suc_rate[i] = self.device_rnd.randint(50, 100) / 100

        self.speed[0] = self.simulator.drone_speed
        self.network_suc_rate[0] = 1.0
        self.speed[1] = self.simulator.drone_speed / 1.3

def compute_circle_path(radius : int, center : tuple) -> list:
    """ compute a set of finite coordinates to simulate a circle trajectory of input radius around a given center

        radius : int -> the radius of the trajectory
        centers : tuple (x, y) the center of the trajectory
        return a list of tuple (coordinates)
    """
    x = list(range(-radius, radius))
    coords = []
    for x_ in x:
        y_ = radius ** 2 - (x_) ** 2
        coords.append((x_, (y_ ** (0.5))))
    coords2 = coords[::-1]
    coords2 = [(x, -y) for x,y in coords2]
    coords += coords2
    return [(x + center[0], y + center[1]) for x,y in coords]

def date():
    return str(time.strftime("%d%m%Y-%H%M%S"))

def euclidean_distance(p1, p2):
    """ Given points p1, p2 in R^2 it returns the norm of the vector connecting them.  """
    dist = ((p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2) ** 0.5
    return dist


def pickle_data(data, filename):
    """ save the metrics on file """
    with open(filename, 'wb') as out:
        pickle.dump(data, out)


def unpickle_data(filename):
    """ load the metrics from a file """
    with open(filename, 'rb') as handle:
        obj = pickle.load(handle)
    return obj


def save_txt(text, file):
    with open(file, "w") as f:
        f.write(text)


# ------------------ Event (Traffic) Generator ----------------------
class EventGenerator:

    def __init__(self, simulator):
        """
        :param simulator: the main simulator object
        """
        self.simulator = simulator
        self.rnd_drones = np.random.RandomState(self.simulator.seed)
        # for now no random on number of event generated
        # self.rnd_event = np.random.RandomState(self.simulator.seed)

    def handle_events_generation(self, cur_step : int, drones : list):
        """
        at fixed time randomly select a drone from the list and sample on it a packet/event.

        :param cur_step: the current step of the simulation to decide whenever sample an event or not
        :param drones: the drones where to sample the event
        :return: nothing
        """
        if cur_step % self.simulator.event_generation_delay == 0:  # if it's time to generate a new packet
            # drone that will receive the packet:
            drone_index = self.rnd_drones.randint(0, len(drones))
            drone = drones[drone_index]
            drone.feel_event(cur_step)

# ------------------ Path manager ----------------------
class PathManager:

    def __init__(self, path_from_json : bool, json_file: str, seed: int, rnd_state=None):
        """
            path_from_json : wheter generate or load the paths for the drones
            json file to read for take the paths of drones
            We assume json_file.format(seed)
        """
        self.path_from_json = path_from_json
        self.json_file = json_file.format(seed)
        if path_from_json:
            self.path_dict = json_to_paths(self.json_file)
            self.rnd_paths = rnd_state
        else:
            self.path_dict = None
            self.rnd_paths = rnd_state

    def path(self, drone_id, simulator, residual_energy):
        """ takes the drone id and
            returns a path (list of tuple)
            for it.

            Notice that: the path can last
            less or more than the simulation.
            In the first case the path should be repeated.
        """
        if config.DEMO_PATH:  # some demo paths
            return self.__demo_path(drone_id)
        if config.CIRCLE_PATH:
            return self.__cirlce_path(drone_id, simulator)
        elif self.path_from_json:  # paths from loaded json
            return self.path_dict[drone_id]
        else:  # generate dynamic paths
            return random_waypoint_generation.get_tour(residual_energy, simulator.env_width,
                                                       simulator.depot_coordinates, index=drone_id,
                                                       random_generator=self.rnd_paths,
                                                       range_decision=config.RANDOM_STEPS,
                                                       random_starting_point=config.RANDOM_START_POINT)

    def __cirlce_path(self, drone_id, simulator, center=None, radius=None):
        if center is None:
            center = simulator.depot_coordinates
        if radius is None:
            radius = simulator.depot_com_range - 10
        n_drones = simulator.n_drones
        traj = compute_circle_path(radius, center)
        step_start = int(len(traj) / n_drones)
        return traj[(drone_id*step_start):] + traj[:(drone_id*step_start)]

    def __demo_path(self, drone_id):
        """ Add handcrafted torus here.  """
        tmp_path = {0: [(750, 750),  (760, 750), (750, 750), (760, 750), (770, 750)],
                    1: [(1280, 80),  (460, 1050), (1060, 1050), (1060, 450), (460, 450), (0, 1500)],
                    2: [(1320, 120), (460, 1050), (1060, 1050), (1060, 450), (460, 450), (0, 1500)],
                    3: [(1400, 160), (460, 1050), (1060, 1050), (1060, 450), (460, 450), (0, 1500)],
                    4: [(1500, 200), (460, 1050), (1060, 1050), (1060, 450), (460, 450), (0, 1500)]}
        return tmp_path[drone_id]

# [(-100, 0.0), (-99, 14.106735979665885), (-98, 19.8997487421324), (-97, 24.310491562286437), (-96, 28.0), (-95, 31.22498999199199), (-94, 34.11744421846396), (-93, 36.75595189897821), (-92, 39.191835884530846), (-91, 41.46082488325576), (-90, 43.58898943540674), (-89, 45.59605246071199), (-88, 47.49736834815167), (-87, 49.3051721424842), (-86, 51.02940328869229), (-85, 52.67826876426369), (-84, 54.258639865002145), (-83, 55.776339069537364), (-82, 57.23635208501674), (-81, 58.642987645583), (-80, 60.0), (-79, 61.310684223877324), (-78, 62.57795138864806), (-77, 63.80438856379708), (-76, 64.99230723708769), (-75, 66.14378277661477), (-74, 67.26068688320095), (-73, 68.3447144993671), (-72, 69.39740629158989), (-71, 70.42016756583301), (-70, 71.4142842854285), (-69, 72.38093671679029), (-68, 73.32121111929344), (-67, 74.23610981186985), (-66, 75.1265598839718), (-65, 75.99342076785332), (-64, 76.83749084919418), (-63, 77.6595132614157), (-62, 78.46018098373213), (-61, 79.24014134263012), (-60, 80.0), (-59, 80.7403244977378), (-58, 81.4616474176652), (-57, 82.16446920658588), (-56, 82.84926070883192), (-55, 83.51646544245033), (-54, 84.16650165000326), (-53, 84.79976415061542), (-52, 85.41662601625049), (-51, 86.01744009211156), (-50, 86.60254037844386), (-49, 87.17224328879004), (-48, 87.72684879784524), (-47, 88.26664149042944), (-46, 88.79189152169245), (-45, 89.30285549745876), (-44, 89.7997772825746), (-43, 90.28288874421332), (-42, 90.75241043630741), (-41, 91.20855223058855), (-40, 91.6515138991168), (-39, 92.08148565265441), (-38, 92.49864863877742), (-37, 92.90317540321213), (-36, 93.29523031752481), (-35, 93.67496997597597), (-34, 94.04254356406997), (-33, 94.39809320108114), (-32, 94.74175425861608), (-31, 95.07365565707464), (-30, 95.39392014169457), (-29, 95.7026645397086), (-28, 96.0), (-27, 96.28603221651622), (-26, 96.56086163658648), (-25, 96.82458365518542), (-24, 97.07728879609277), (-23, 97.31906288081488), (-22, 97.54998718605759), (-21, 97.77013859047148), (-20, 97.97958971132712), (-19, 98.17840903172143), (-18, 98.3666610188635), (-17, 98.54440623394105), (-16, 98.71170143402453), (-15, 98.86859966642594), (-14, 99.0151503558925), (-13, 99.15139938498095), (-12, 99.27738916792686), (-11, 99.39315871829409), (-10, 99.498743710662), (-9, 99.5941765365827), (-8, 99.67948635501689), (-7, 99.75469913743412), (-6, 99.81983770774224), (-5, 99.87492177719089), (-4, 99.91996797437437), (-3, 99.95498987044118), (-2, 99.9799979995999), (-1, 99.99499987499375), (0, 100.0), (1, 99.99499987499375), (2, 99.9799979995999), (3, 99.95498987044118), (4, 99.91996797437437), (5, 99.87492177719089), (6, 99.81983770774224), (7, 99.75469913743412), (8, 99.67948635501689), (9, 99.5941765365827), (10, 99.498743710662), (11, 99.39315871829409), (12, 99.27738916792686), (13, 99.15139938498095), (14, 99.0151503558925), (15, 98.86859966642594), (16, 98.71170143402453), (17, 98.54440623394105), (18, 98.3666610188635), (19, 98.17840903172143), (20, 97.97958971132712), (21, 97.77013859047148), (22, 97.54998718605759), (23, 97.31906288081488), (24, 97.07728879609277), (25, 96.82458365518542), (26, 96.56086163658648), (27, 96.28603221651622), (28, 96.0), (29, 95.7026645397086), (30, 95.39392014169457), (31, 95.07365565707464), (32, 94.74175425861608), (33, 94.39809320108114), (34, 94.04254356406997), (35, 93.67496997597597), (36, 93.29523031752481), (37, 92.90317540321213), (38, 92.49864863877742), (39, 92.08148565265441), (40, 91.6515138991168), (41, 91.20855223058855), (42, 90.75241043630741), (43, 90.28288874421332), (44, 89.7997772825746), (45, 89.30285549745876), (46, 88.79189152169245), (47, 88.26664149042944), (48, 87.72684879784524), (49, 87.17224328879004), (50, 86.60254037844386), (51, 86.01744009211156), (52, 85.41662601625049), (53, 84.79976415061542), (54, 84.16650165000326), (55, 83.51646544245033), (56, 82.84926070883192), (57, 82.16446920658588), (58, 81.4616474176652), (59, 80.7403244977378), (60, 80.0), (61, 79.24014134263012), (62, 78.46018098373213), (63, 77.6595132614157), (64, 76.83749084919418), (65, 75.99342076785332), (66, 75.1265598839718), (67, 74.23610981186985), (68, 73.32121111929344), (69, 72.38093671679029), (70, 71.4142842854285), (71, 70.42016756583301), (72, 69.39740629158989), (73, 68.3447144993671), (74, 67.26068688320095), (75, 66.14378277661477), (76, 64.99230723708769), (77, 63.80438856379708), (78, 62.57795138864806), (79, 61.310684223877324), (80, 60.0), (81, 58.642987645583), (82, 57.23635208501674), (83, 55.776339069537364), (84, 54.258639865002145), (85, 52.67826876426369), (86, 51.02940328869229), (87, 49.3051721424842), (88, 47.49736834815167), (89, 45.59605246071199), (90, 43.58898943540674), (91, 41.46082488325576), (92, 39.191835884530846), (93, 36.75595189897821), (94, 34.11744421846396), (95, 31.22498999199199), (96, 28.0), (97, 24.310491562286437), (98, 19.8997487421324), (99, 14.106735979665885), (99, -14.106735979665885), (98, -19.8997487421324), (97, -24.310491562286437), (96, -28.0), (95, -31.22498999199199), (94, -34.11744421846396), (93, -36.75595189897821), (92, -39.191835884530846), (91, -41.46082488325576), (90, -43.58898943540674), (89, -45.59605246071199), (88, -47.49736834815167), (87, -49.3051721424842), (86, -51.02940328869229), (85, -52.67826876426369), (84, -54.258639865002145), (83, -55.776339069537364), (82, -57.23635208501674), (81, -58.642987645583), (80, -60.0), (79, -61.310684223877324), (78, -62.57795138864806), (77, -63.80438856379708), (76, -64.99230723708769), (75, -66.14378277661477), (74, -67.26068688320095), (73, -68.3447144993671), (72, -69.39740629158989), (71, -70.42016756583301), (70, -71.4142842854285), (69, -72.38093671679029), (68, -73.32121111929344), (67, -74.23610981186985), (66, -75.1265598839718), (65, -75.99342076785332), (64, -76.83749084919418), (63, -77.6595132614157), (62, -78.46018098373213), (61, -79.24014134263012), (60, -80.0), (59, -80.7403244977378), (58, -81.4616474176652), (57, -82.16446920658588), (56, -82.84926070883192), (55, -83.51646544245033), (54, -84.16650165000326), (53, -84.79976415061542), (52, -85.41662601625049), (51, -86.01744009211156), (50, -86.60254037844386), (49, -87.17224328879004), (48, -87.72684879784524), (47, -88.26664149042944), (46, -88.79189152169245), (45, -89.30285549745876), (44, -89.7997772825746), (43, -90.28288874421332), (42, -90.75241043630741), (41, -91.20855223058855), (40, -91.6515138991168), (39, -92.08148565265441), (38, -92.49864863877742), (37, -92.90317540321213), (36, -93.29523031752481), (35, -93.67496997597597), (34, -94.04254356406997), (33, -94.39809320108114), (32, -94.74175425861608), (31, -95.07365565707464), (30, -95.39392014169457), (29, -95.7026645397086), (28, -96.0), (27, -96.28603221651622), (26, -96.56086163658648), (25, -96.82458365518542), (24, -97.07728879609277), (23, -97.31906288081488), (22, -97.54998718605759), (21, -97.77013859047148), (20, -97.97958971132712), (19, -98.17840903172143), (18, -98.3666610188635), (17, -98.54440623394105), (16, -98.71170143402453), (15, -98.86859966642594), (14, -99.0151503558925), (13, -99.15139938498095), (12, -99.27738916792686), (11, -99.39315871829409), (10, -99.498743710662), (9, -99.5941765365827), (8, -99.67948635501689), (7, -99.75469913743412), (6, -99.81983770774224), (5, -99.87492177719089), (4, -99.91996797437437), (3, -99.95498987044118), (2, -99.9799979995999), (1, -99.99499987499375), (0, -100.0), (-1, -99.99499987499375), (-2, -99.9799979995999), (-3, -99.95498987044118), (-4, -99.91996797437437), (-5, -99.87492177719089), (-6, -99.81983770774224), (-7, -99.75469913743412), (-8, -99.67948635501689), (-9, -99.5941765365827), (-10, -99.498743710662), (-11, -99.39315871829409), (-12, -99.27738916792686), (-13, -99.15139938498095), (-14, -99.0151503558925), (-15, -98.86859966642594), (-16, -98.71170143402453), (-17, -98.54440623394105), (-18, -98.3666610188635), (-19, -98.17840903172143), (-20, -97.97958971132712), (-21, -97.77013859047148), (-22, -97.54998718605759), (-23, -97.31906288081488), (-24, -97.07728879609277), (-25, -96.82458365518542), (-26, -96.56086163658648), (-27, -96.28603221651622), (-28, -96.0), (-29, -95.7026645397086), (-30, -95.39392014169457), (-31, -95.07365565707464), (-32, -94.74175425861608), (-33, -94.39809320108114), (-34, -94.04254356406997), (-35, -93.67496997597597), (-36, -93.29523031752481), (-37, -92.90317540321213), (-38, -92.49864863877742), (-39, -92.08148565265441), (-40, -91.6515138991168), (-41, -91.20855223058855), (-42, -90.75241043630741), (-43, -90.28288874421332), (-44, -89.7997772825746), (-45, -89.30285549745876), (-46, -88.79189152169245), (-47, -88.26664149042944), (-48, -87.72684879784524), (-49, -87.17224328879004), (-50, -86.60254037844386), (-51, -86.01744009211156), (-52, -85.41662601625049), (-53, -84.79976415061542), (-54, -84.16650165000326), (-55, -83.51646544245033), (-56, -82.84926070883192), (-57, -82.16446920658588), (-58, -81.4616474176652), (-59, -80.7403244977378), (-60, -80.0), (-61, -79.24014134263012), (-62, -78.46018098373213), (-63, -77.6595132614157), (-64, -76.83749084919418), (-65, -75.99342076785332), (-66, -75.1265598839718), (-67, -74.23610981186985), (-68, -73.32121111929344), (-69, -72.38093671679029), (-70, -71.4142842854285), (-71, -70.42016756583301), (-72, -69.39740629158989), (-73, -68.3447144993671), (-74, -67.26068688320095), (-75, -66.14378277661477), (-76, -64.99230723708769), (-77, -63.80438856379708), (-78, -62.57795138864806), (-79, -61.310684223877324), (-80, -60.0), (-81, -58.642987645583), (-82, -57.23635208501674), (-83, -55.776339069537364), (-84, -54.258639865002145), (-85, -52.67826876426369), (-86, -51.02940328869229), (-87, -49.3051721424842), (-88, -47.49736834815167), (-89, -45.59605246071199), (-90, -43.58898943540674), (-91, -41.46082488325576), (-92, -39.191835884530846), (-93, -36.75595189897821), (-94, -34.11744421846396), (-95, -31.22498999199199), (-96, -28.0), (-97, -24.310491562286437), (-98, -19.8997487421324), (-99, -14.106735979665885), (-100, -0.0)],


def json_to_paths(json_file_path):
    """ load the tour for drones
        and return a dictionary {drone_id : list of waypoint}

        e.g.,
        accept json that contains:
        {"drones": [{"index": "0", "tour": ["(1500, 0)", "(1637, 172)", ...
                    (1500, 0)"]}, {"index": "1", "tour": ["(1500, 0)",

        TOURS = {
            0 : [(0,0), (2000,2000), (1500, 1500), (200, 2000)],
            1 : [(0,0), (2000, 200), (200, 2000), (1500, 1500)]
        }
    """
    out_data = {}
    with open(json_file_path, 'r') as in_file:
        data = json.load(in_file)
        for drone_data in data["drones"]:
            drone_index = int(drone_data["index"])
            drone_path = []
            for waypoint in drone_data["tour"]:
                drone_path.append(make_tuple(waypoint))
            out_data[drone_index] = drone_path
    return out_data


class LimitedList:
    """ Time window """
    def __init__(self, threshold=None):
        self.llist = []
        self.threshold = threshold

    def append(self, el):
        if self.threshold and self.threshold < len(self.llist) + 1:
            self.llist = self.llist[1:]
        self.llist.append(el)

    def __len__(self):
        return len(self.llist)

    def __getitem__(self, index):
        return self.llist[index]


def make_path(fname):
    path = pathlib.Path(fname)
    path.parent.mkdir(parents=True, exist_ok=True)


def plot_X(X, plt_title, plt_path, window_size=30, is_avg=True):
    if len(X) >= window_size:
        df = pd.Series(X)
        scatter_print = X[window_size:]
        to_plot_data = df.rolling(window_size).mean()[window_size:]

        plt.clf()
        plt.plot(range(len(scatter_print)), to_plot_data, label="Moving Average-" + str(window_size))
        if is_avg:
            plt.plot(range(len(scatter_print)), [np.average(scatter_print)] * len(scatter_print), label="avg")

        plt.legend()
        plt.title(plt_title)
        plt.savefig(plt_path)
        plt.clf()


""" This class handle the return to depot for
    the drones, such that they return to the depot in a coordinated fashion
    currently is based on channel -> in future can also handle cluster head/waypoints
"""

class PathToDepot():

    def __init__(self, x_position, simulator):
        """ for now just a middle channel in the area used by all the drones """
        self.x_position = x_position
        self.simulator = simulator

    def next_target(self, drone_pos):
        """ based on the drone position return the next target:
            |-> channel position or cluster head position
            |-> the depot if the drones are already in the channel or have overpass the cluster head
        """
        # only channel mode
        if abs(drone_pos[
                   0] - self.x_position) < 1:  # the drone is already on the channel with an error of 1 meter
            return self.simulator.depot_coordinates
        else:
            return self.x_position, drone_pos[1]  # the closest point to the channel


def measure_scaler(measure, dom_start, dom_target):
    """ Scales the measure value in the start domain [Type, min, max], in the target domain. """
    return (measure - dom_start[1]) / (dom_start[2] - dom_start[1]) * (dom_target[2] - dom_target[1]) + dom_target[1]

# -------------------- all cells computation ---------------------#

class TraversedCells:

    @staticmethod
    def cells_in_travel(size_cell, width_area, start, end):
        """ return the cell number in which the pos (x, y) lay """

        start_cell, coords_cell_start = TraversedCells.coord_to_cell(size_cell, width_area, start[0], start[1])  # int, lower left coordinates
        end_cell, coords_cell_end = TraversedCells.coord_to_cell(size_cell, width_area, end[0], end[1])

        out_cells = []

        if coords_cell_end[1] == coords_cell_start[1]:  # vertical alignment
            min_x = min(coords_cell_start[0], coords_cell_end[0])
            max_x = max(coords_cell_start[0], coords_cell_end[0])
            for x_ in range(min_x, max_x + 1):
                out_cells.append((x_, coords_cell_end[1]))
            return out_cells

        if coords_cell_end[0] == coords_cell_start[0]:  # horizontal alignment
            min_y = min(coords_cell_start[1], coords_cell_end[1])
            max_y = max(coords_cell_start[1], coords_cell_end[1])
            for y_ in range(min_y, max_y + 1):
                out_cells.append((coords_cell_end[0], y_))
            return out_cells

        # Diagonal line
        # Boundaries of the rectangle
        min_x, max_x = min(coords_cell_start[0], coords_cell_end[0]), max(coords_cell_start[0], coords_cell_end[0])
        min_y, max_y = min(coords_cell_start[1], coords_cell_end[1]), max(coords_cell_start[1], coords_cell_end[1])

        # All the cells of the rectangle, indices
        coords_index = [(i, j) for i in range(min_x, max_x+1) for j in range(min_y, max_y+1)]
        for cell in coords_index:

            ll = cell[0]*size_cell, cell[1]*size_cell
            lr = cell[0]*size_cell + size_cell, cell[1]*size_cell
            ul = cell[0]*size_cell, cell[1]*size_cell + size_cell
            ur = cell[0]*size_cell + size_cell, cell[1]*size_cell + size_cell

            if TraversedCells.intersect_quad(start, end, ll, lr, ul, ur):
                out_cells.append(cell)

        return out_cells  # list of lower-lefts

    @staticmethod
    def intersect_quad(start, end, ll, lr, ul, ur):

        return (TraversedCells.intersect_segments(start, end, ll, lr)
                or TraversedCells.intersect_segments(start, end, ul, ur)
                or TraversedCells.intersect_segments(start, end, ul, ll)
                or TraversedCells.intersect_segments(start, end, lr, ur))

    @staticmethod
    def intersect_segments(start1:tuple, end1:tuple, start2:tuple, end2:tuple):
        if end1 == start2:
            return True
        if end2 == start1:
            return True
        if start2 == start1:
            return True
        if end2 == end1:
            return True

        a = np.asarray(end1) - np.asarray(start1)  # direction of line a
        b = np.asarray(start2) - np.asarray(end2)  # direction of line b, reversed
        d = np.asarray(start2) - np.asarray(start1)  # right-hand side
        det = a[0] * b[1] - a[1] * b[0]

        if det == 0:
            return False

        t = (a[0] * d[1] - a[1] * d[0]) / det
        return 0 <= t <= 1

    @staticmethod
    def all_centers(widht_area, height_area, size_cell):
        """ return all cell along their centers """
        all_cells_and_centers = []
        for x in range(0, widht_area, size_cell):
            for y in range(0, height_area, size_cell):
                all_cells_and_centers.append(
                    (TraversedCells.coord_to_cell(size_cell, widht_area, x, y),
                        (x + (size_cell/2.0), y + (size_cell/2.0)))
                )
        return all_cells_and_centers

    @staticmethod
    def coord_to_cell(size_cell, width_area, x_pos, y_pos):
        """ return the cell number in which the pos (x"abs", y"abs") lay """
        x_cell_coords = int(x_pos / size_cell)
        y_cell_coords = int(y_pos / size_cell)
        return TraversedCells.cell_coord_to_cell_number(size_cell, width_area, x_cell_coords,
                                                        y_cell_coords), (x_cell_coords, y_cell_coords)

    @staticmethod
    def cell_coord_to_cell_number(size_cell, width_area, x_cell_coords, y_cell_coords):
        """ return the number o the cells given the indexes """

        x_cells = np.ceil(width_area / size_cell)  # numero di celle su X
        return x_cell_coords + (x_cells * y_cell_coords)

