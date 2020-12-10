from src.utilities import config
from src.simulation.simulator import Simulator
import os
from argparse import ArgumentParser

LEN_TEST = 18000 # around 3hr of mission

def sim_setup(n_drones, seed, algorithm_routing):
    """ return the sim setup """
    len_simulation=LEN_TEST
    time_step_duration=0.15 #150ms each step -> 72000 * 0.15 -> 3600*3 -> 3hr
    env_width=1500
    env_height=1500
    drone_com_range=200
    drone_sen_range=0
    drone_speed=8 #18km/h
    drone_max_buffer_size=500000
    drone_max_energy=100000
    drone_retransmission_delta=10
    drone_communication_success=0.9 # TODO
    depot_com_range=5
    depot_coordinates=(750, 0)
    event_duration=2000 #circa 5min
    event_generation_prob=0.8
    event_generation_delay=300
    packets_max_ttl=len_simulation
    show_plot=False
    routing_algorithm = config.RoutingAlgorithm[algorithm_routing]
    communication_error_type = config.ChannelError.ON_DEVICE

    model_name = "model_" + str(n_drones)

    return Simulator(
                len_simulation=len_simulation,
                 time_step_duration=time_step_duration,
                 seed=seed,
                 n_drones=n_drones,
                 env_width=env_width,
                 env_height=env_height,

                 drone_com_range=drone_com_range,
                 drone_sen_range=drone_sen_range,
                 drone_speed=drone_speed,
                 drone_max_buffer_size=drone_max_buffer_size,
                 drone_max_energy=drone_max_energy,
                 drone_retransmission_delta=drone_retransmission_delta,
                 drone_communication_success=drone_communication_success,
                 event_generation_delay = event_generation_delay,

                 depot_com_range=depot_com_range,
                 depot_coordinates=depot_coordinates,

                 event_duration=event_duration,
                 event_generation_prob=event_generation_prob,
                 packets_max_ttl=packets_max_ttl,

                 show_plot=show_plot,
                 routing_algorithm=routing_algorithm,
                 communication_error_type=communication_error_type,

                 # ML parameters
                 simulation_name=model_name,

    )

def exp_ndrones(path_filename, n_drones, in_seed, out_seed, algorithm_routing):
    # ---- Experiment 1 ---- #
    # test the routing for 
    # at varying k and seed values
    scores = {}
    for seed in range(in_seed, out_seed):

        print("Running " + algorithm_routing + " with", n_drones, "drones with seed:", seed)

        simulation = sim_setup(n_drones, seed, algorithm_routing)
        simulation.run()

        simulation.save_metrics(path_filename + "out__ndrones_" + str(n_drones) +
                            "_seed" + str(seed) + "_alg_" + algorithm_routing)
        print("Score: ",simulation.score(), algorithm_routing)
        scores[seed] = simulation.score()
        simulation.close()

    print("Ndrones: ", n_drones, " - Algo: ", algorithm_routing, "- Scores: ", scores)

if __name__ == "__main__":
    # parser input
    parser = ArgumentParser()
    
    routing_choices = config.RoutingAlgorithm.keylist()

    # MANDATORY
    parser.add_argument("-nd", dest='number_of_drones', action="store", type=int,
                        help="the number of drones to use in the simulataion")
    parser.add_argument("-i_s", dest='initial_seed', action="store", type=int,
                        help="the initial seed (included) to use in the simualtions")
    parser.add_argument("-e_s", dest='end_seed', action="store", type=int,
                        help="the end seed (excluded) to use in the simualtions"
                             + "-notice that the simulations will run for seed in (i_s, e_s)")
    parser.add_argument("-alg", dest='algorithm_routing', action="store", type=str,
                        choices=routing_choices, help="the routing algorithm to use")


    args = parser.parse_args()

    number_of_drones = args.number_of_drones
    initial_seed = args.initial_seed
    end_seed = args.end_seed
    algorithm_routing = args.algorithm_routing
    path_filename = config.EXPERIMENTS_DIR

    # build directories for results and models
    os.system("mkdir " + path_filename)

    exp_ndrones(path_filename, number_of_drones, initial_seed, end_seed, algorithm_routing)
    print("Sim completed")    

