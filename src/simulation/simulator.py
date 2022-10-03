from src.drawing import pp_draw
from src.entities.uavs.uav_entities import *
from src.simulation.logger import Logger
from src.simulation.metrics import Metrics
from src.utilities import config, utilities
from src.routing_algorithms.net_routing import MediumDispatcher
from collections import defaultdict
from tqdm import tqdm
from src.entities.generic.entity import Environment
import numpy as np
import math
import time

"""
This file contains the Simulation class. It allows to explicit all the relevant parameters of the simulation,
as default all the parameters are set to be those in the config file. For extensive experimental campains, 
you can initialize the Simulator with non default values. 
"""


class Simulator:

    def __init__(self,
                 len_simulation=config.SIM_DURATION,
                 time_step_duration=config.TS_DURATION,
                 seed=config.SEED,
                 n_drones=config.N_DRONES,
                 env_width=config.ENV_WIDTH,
                 env_height=config.ENV_HEIGHT,
                 drone_com_range=config.COMMUNICATION_RANGE_DRONE,
                 drone_sen_range=config.SENSING_RANGE_DRONE,
                 drone_speed=config.DRONE_SPEED,
                 drone_max_buffer_size=config.DRONE_MAX_BUFFER_SIZE,
                 drone_max_energy=config.DRONE_MAX_ENERGY,
                 drone_retransmission_delta=config.RETRANSMISSION_DELAY,
                 drone_communication_success=config.COMMUNICATION_P_SUCCESS,
                 depot_com_range=config.DEPOT_COMMUNICATION_RANGE,
                 depot_coordinates=config.DEPOT_COO,
                 event_duration=config.EVENTS_DURATION,
                 event_generation_prob=config.P_FEEL_EVENT,
                 event_generation_delay=config.D_FEEL_EVENT,
                 packets_max_ttl=config.PACKETS_MAX_TTL,
                 show_plot=config.PLOT_SIM,
                 routing_algorithm=config.ROUTING_ALGORITHM,
                 communication_error_type=config.CHANNEL_ERROR_TYPE,
                 prob_size_cell_r=config.CELL_PROB_SIZE_R,
                 simulation_name=""):
        self.drone_com_range = drone_com_range
        self.drone_sen_range = drone_sen_range
        self.drone_speed = drone_speed
        self.drone_max_buffer_size = drone_max_buffer_size
        self.drone_max_energy = drone_max_energy
        self.drone_retransmission_delta = drone_retransmission_delta
        self.drone_communication_success = drone_communication_success
        self.n_drones = n_drones
        self.env_width = env_width
        self.env_height = env_height
        self.depot_com_range = depot_com_range
        self.depot_coordinates = depot_coordinates
        self.len_simulation = len_simulation
        self.time_step_duration = time_step_duration
        self.seed = seed
        self.event_duration = event_duration
        self.event_max_retrasmission = math.ceil(event_duration / drone_retransmission_delta)  # 600 esempio
        self.event_generation_prob = event_generation_prob
        self.event_generation_delay = event_generation_delay
        self.packets_max_ttl = packets_max_ttl
        self.show_plot = show_plot
        self.routing_algorithm = routing_algorithm
        self.communication_error_type = communication_error_type
        self.cur_step = 0

        # --------------- cell for drones -------------
        self.prob_size_cell_r = prob_size_cell_r
        self.prob_size_cell = int(self.drone_com_range * self.prob_size_cell_r)
        self.cell_prob_map = defaultdict(lambda: [0, 0, 0])

        self.sim_save_file = config.SAVE_PLOT_DIR + self.__sim_name()
        self.path_to_depot = None

        # Setup vari
        # for stats
        self.metrics = Metrics()
        self.logger = Logger()

        # setup network
        self.__setup_net_dispatcher()

        # Set up the simulation
        self.__set_simulation()

        self.simulation_name = "test-" + utilities.date() + "_" + str(simulation_name) + "_" + str(self.seed)
        self.simulation_test_dir = self.simulation_name + "/"

        self.start = time.time()
        self.event_generator = utilities.EventGenerator(self)

    def __setup_net_dispatcher(self):
        """

        @return:
        """

        self.network_dispatcher = MediumDispatcher(self)


    def __set_random_generators(self):
        if self.seed is not None:
            self.rnd_network = np.random.RandomState(self.seed)
            self.rnd_routing = np.random.RandomState(self.seed)
            self.rnd_env = np.random.RandomState(self.seed)
            self.rnd_event = np.random.RandomState(self.seed)

    def __set_simulation(self):
        """ the method creates all the uav entities """

        self.__set_random_generators()
        self.path_manager = utilities.PathManager(config.PATH_FROM_JSON, config.JSONS_PATH_PREFIX, self.seed)
        self.environment = Environment(self, self.env_width, self.env_height)

        self.depot = Depot(simulator=self,
                           coordinates=self.depot_coordinates,
                           communication_range=self.depot_com_range)

        self.drones = []

        # drone 0 is the first
        for i in range(self.n_drones):

            self.drones.append(Drone(self, i, self.path_manager.path(i, self), self.depot))

        self.environment.drones = self.drones
        self.environment.depot = self.depot

        # Set the maximum distance between the drones and the depot
        self.max_dist_drone_depot = utilities.euclidean_distance(self.depot.coordinates, (self.env_width, self.env_height))

        if self.show_plot or config.SAVE_PLOT:
            self.draw_manager = pp_draw.PathPlanningDrawer(self.environment, self, borders=True)

    def __sim_name(self):
        """
        Returns the identification name for
        the current simulation. It is useful to print
        the simulation progress
        @return:
        """

        return f"SIMULATION_SEED: {str(self.seed)} drones: {str(self.n_drones)} "

    def __plot(self):
        """ plot the simulation """

        if self.cur_step % config.SKIP_SIM_STEP != 0:
            return

        # delay draw
        if config.WAIT_SIM_STEP > 0:

            time.sleep(config.WAIT_SIM_STEP)

        # drones plot
        for drone in self.drones:

            self.draw_manager.draw_drone(drone, self.cur_step)

        # depot plot
        self.draw_manager.draw_depot(self.depot)

        # events
        for event in self.environment.active_events:

            self.draw_manager.draw_event(event)

        # draw simulation info
        self.draw_manager.draw_simulation_info(cur_step=self.cur_step, max_steps=self.len_simulation)

        # rendering phase
        file_name = self.sim_save_file + str(self.cur_step) + ".png"
        self.draw_manager.update(show=self.show_plot, save=config.SAVE_PLOT, filename=file_name)

    def increase_meetings_probs(self, drones, cur_step):
        """ Increases the probabilities of meeting someone. """
        cells = set()
        for drone in drones:
            coords = drone.coordinates
            cell_index = utilities.TraversedCells.coord_to_cell(size_cell=self.prob_size_cell,
                                                                width_area=self.env_width,
                                                                x_pos=coords[0],  # e.g. 1500
                                                                y_pos=coords[1])  # e.g. 500
            cells.add(int(cell_index[0]))

        for cell, cell_center in utilities.TraversedCells.all_centers(self.env_width, self.env_height,
                                                                          self.prob_size_cell):

            index_cell = int(cell[0])
            old_vals = self.cell_prob_map[index_cell]

            if index_cell in cells:
                old_vals[0] += 1

            old_vals[1] = cur_step + 1
            old_vals[2] = old_vals[0] / max(1, old_vals[1])
            self.cell_prob_map[index_cell] = old_vals

    def run(self):
        """
        the method starts the simulation
        """

        for cur_step in tqdm(range(self.len_simulation)):

            self.cur_step = cur_step
            # check for new events and remove the expired ones from the environment
            # self.environment.update_events(cur_step)
            # sense the area and move drones and sense the area
            self.network_dispatcher.run_medium(cur_step)

            # generates events
            # sense the events
            self.event_generator.handle_events_generation(cur_step, self.drones)

            for drone in self.drones:

                # 1. update expired packets on drone buffers
                # 2. try routing packets vs other drones or depot
                # 3. actually move the drone towards next waypoint or depot

                drone.update_packets()
                drone.routing(self.drones)
                drone.move(self.time_step_duration)

            # in case we need probability map
            if config.ENABLE_PROBABILITIES:

                self.increase_meetings_probs(self.drones, cur_step)

            if self.show_plot or config.SAVE_PLOT:
                self.__plot()

        if config.DEBUG:

            print("End of simulation, sim time: " + str((self.cur_step + 1) * self.time_step_duration) + " sec, #iteration: " + str(cur_step + 1))

    def close(self):
        """ do some stuff at the end of simulation"""
        print("Closing simulation")

        self.compute_final_metrics()
        self.print_metrics(metrics=True, logger=False)
        self.save_metrics(config.ROOT_EVALUATION_DATA + self.simulation_name)

    def compute_final_metrics(self):
        """

        @return:
        """
        self.metrics.drones_packets_to_depot = len(self.logger.drones_packets_to_depot)
        self.metrics.all_packets_correctly_sent_by_drones = len(self.logger.drones_packets)

    def print_metrics(self, metrics: bool = True, logger: bool = False):
        """

        @return:
        """
        if metrics:

            print(self.metrics)

        if logger:

            print(self.logger)

    def save_metrics(self, filename_path, save_pickle=False):
        """ add signature """
        self.metrics.save_as_json(filename_path + ".json")
        if save_pickle:
            self.metrics.save(filename_path + ".pickle")

    def score(self):
        """ returns a score for the exectued simulation: 

                sum( event delays )  / number of events

            Notice that, expired or not found events will be counted with a max_delay
        """
        score = round(self.metrics.score(), 2)
        print("Score sim " + self.simulation_name + ":", score)
        return score
