
from src.drawing import pp_draw
from src.entities.uav_entities import *
from src.simulation.metrics import Metrics
from src.utilities import config, utilities
from src.routing_algorithms.net_routing import MediumDispatcher
from collections import defaultdict

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
        self.restart_mission = set()

        # --------------- cell for drones -------------
        self.prob_size_cell_r = prob_size_cell_r
        self.prob_size_cell = int(self.drone_com_range * self.prob_size_cell_r)
        self.cell_prob_map = defaultdict(lambda: [0, 0, 0])

        self.sim_save_file = config.SAVE_PLOT_DIR + self.__sim_name()
        self.path_to_depot = None

        # Setup vari
        # for stats
        self.metrics = Metrics(self)

        # setup network
        self.__setup_net_dispatcher()

        # Setup the simulation
        self.__set_simulation()
        self.__set_metrics()

        self.simulation_name = "test-" + utilities.date() + "_" + str(simulation_name) + "_" + str(self.seed)
        self.simulation_test_dir = self.simulation_name + "/"

        self.start = time.time()
        self.event_generator = utilities.EventGenerator(self)

    def __setup_net_dispatcher(self):
        self.network_dispatcher = MediumDispatcher(self.metrics)

    def __set_metrics(self):
        """ the method sets up all the parameters in the metrics class """
        self.metrics.info_mission()

    def __set_random_generators(self):
        self.drone_capabilities = utilities.DroneCapabilities(self)
        if self.seed is not None:
            self.rnd_network = np.random.RandomState(self.seed)
            self.rnd_routing = np.random.RandomState(self.seed)
            self.rnd_env = np.random.RandomState(self.seed)
            self.rnd_path = np.random.RandomState(self.seed)
            self.rnd_event = np.random.RandomState(self.seed)


    def __set_simulation(self):
        """ the method creates all the uav entities """

        self.__set_random_generators()
        self.environment = Environment(self.env_width, self.env_height, self)

        self.depot = Depot(self.depot_coordinates, self.depot_com_range, self)

        self.drones = []

        # drone 0 is the first
        for i in range(self.n_drones):
            self.drones.append(Drone(i, None, self.drone_capabilities.speed[i],
                                    self.drone_capabilities.network_suc_rate[i], self.depot, self))

        self.environment.add_drones(self.drones)
        self.environment.add_depot(self.depot)

        # Set the maximum distance between the drones and the depot
        self.max_dist_drone_depot = utilities.euclidean_distance(self.depot.coords, (self.env_width, self.env_height))

        if self.show_plot or config.SAVE_PLOT:
            self.draw_manager = pp_draw.PathPlanningDrawer(self.environment, self, borders=True)


    def __sim_name(self):
        """
            return the identification name for
            the current simulation. It is useful to print
            the simulation progress
        """
        return "sim_seed" + str(self.seed) + "drones" + str(self.n_drones) + "_step"

    def __plot(self, cur_step):
        """ plot the simulation """
        if cur_step % config.SKIP_SIM_STEP != 0:
            return

        # delay draw
        if config.WAIT_SIM_STEP > 0:
            time.sleep(config.WAIT_SIM_STEP)

        # drones plot
        for drone in self.drones:
            self.draw_manager.draw_drone(drone, cur_step)

        # depot plot
        self.draw_manager.draw_depot(self.depot)

        # events
        for event in self.environment.active_events:
            self.draw_manager.draw_event(event)

        # Draw simulation info
        self.draw_manager.draw_simulation_info(cur_step=cur_step, max_steps=self.len_simulation)

        # rendering
        self.draw_manager.update(show=self.show_plot, save=config.SAVE_PLOT, filename=self.sim_save_file + str(cur_step) + ".png")

    def increase_meetings_probs(self, drones, cur_step):
        """ Increases the probabilities of meeting someone. """
        cells = set()
        for drone in drones:
            coords = drone.coords
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
        """ the method starts the simulation """
        cells_to_travel = None
        for cur_step in range(self.len_simulation):
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

                drone.update_packets(cur_step)
                drone.routing(self.drones, self.depot, cur_step)
                drone.move(self.time_step_duration)

            if len(self.restart_mission) == len(self.drones) and self.drones[0].coords != self.depot_coordinates:
                self.restart_mission = set()
            # in case we need probability map
            if config.ENABLE_PROBABILITIES:
                self.increase_meetings_probs(self.drones, cur_step)

            if cur_step % 10000 == 0:
                end = time.time()
                print("step: " + str(cur_step), time.strftime("%H:%M:%S", time.gmtime(end - self.start)))
                self.start = time.time()

            if self.show_plot or config.SAVE_PLOT:
                self.__plot(cur_step)

        if config.DEBUG:
            print("End of simulation, sim time: " + str((cur_step + 1) * self.time_step_duration) + " sec, #iteration: " + str(cur_step + 1))

    def close(self):
        """ do some stuff at the end of simulation"""
        print("Closing simulation")
        if self.routing_algorithm.name == "AI":
            self.drones[0].routing_algorithm.print()
        self.print_metrics(plot_id="final")
        self.save_metrics(config.ROOT_EVALUATION_DATA + self.simulation_name)

    def print_metrics(self, plot_id="final"):
        """ add signature """
        self.metrics.print_overall_stats()

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
