import argparse
import os
from multiprocessing import Pool
import datetime as dt
import numpy as np


CODE_TO_RUN = "python3 -m src.experiments.experiment_ndrones -nd {} -i_s {} -e_s {} -alg {}"
PLOT_CODE = "python3 -m src.experiments.json_and_plot -i_s {} -e_s {} "
NUM_CORE = 8

def run_in_parallel(algorithms, drones, seeds, num_core, code):
    processes = []
    for algo in algorithms:
        for ndr in drones:
            for i in range(seeds):
                processes.append(code.format(ndr, i, i + 1, algo))
    pool = Pool(processes=num_core)
    pool.map(run_process, processes)

def run_process(process):
    os.system(process)

if __name__ == "__main__":
    # execution
    ALGO_TO_RUN = ["AI", "MGEO", "GEO"] # run algorithms
    NDRONES = [2, 5, 10] # 15, 20, 30, 40
    NSEEDS = 4  # 30
    #run_in_parallel(ALGO_TO_RUN, NDRONES, NSEEDS, NUM_CORE, CODE_TO_RUN)

    # plot
    algo_command = "-exp_suffix "
    ndrones_command = "-nd "
    plot_code = PLOT_CODE.format(0, NSEEDS)
    for ndr in NDRONES:
        plot_code += " " + ndrones_command + str(ndr)

    for algo in ALGO_TO_RUN:
        plot_code += " " + algo_command + str(algo)

    os.system(plot_code)