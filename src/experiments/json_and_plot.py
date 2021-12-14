""" the file reads the json from the simulations, 
    summarizes the data (e.g., mean, std, sum etc...)
    and plot the results 
"""
import matplotlib.pyplot as plt 
import json 
import numpy as np
import matplotlib.patches as mpatches
import collections
import matplotlib
from src.utilities import config


from argparse import ArgumentParser


#matplotlib size of text
LABEL_SIZE = 16
LEGEND_SIZE = 14
TITLE_SIZE = 26
ALL_SIZE = 14

def plot_coverage_distribution(filename_format : list, ndrones :list, metric : str, 
                            alg_ritrasmission : list, seeds :list, size_mission : int):
    """ plot for varying ndrones """
    for nd in ndrones:
        out_data = {}
        for alg_k in alg_ritrasmission:
            out_data[alg_k] = coverage_distribution(filename_format, nd, alg_k, seeds)
        
        fig, ((ax1, ax2, ax3), (ax4, ax5, ax6), (ax7, ax8, ax9)) = plt.subplots(3, 3)
        fig.set_size_inches(16, 10)

        fig.suptitle('Plot distribution of coverage by' + str(nd) + 'drones')
        ax_n = [ax1, ax2, ax3, ax4, ax5, ax6, ax7, ax8, ax9]
        for i in range(len(alg_ritrasmission)):
            alg_k = alg_ritrasmission[i]
            X, Y = out_data[alg_k]
            ax_n[i].set_title(alg_k)
            ax_n[i].scatter(X, Y, s=1)
            ax_n[i].set_xlim((0, size_mission))
            ax_n[i].set_ylim((0, size_mission))
        
        plt.tight_layout()
        plt.savefig(metric + "_" + str(nd) + "_.png")
        #plt.show()
        plt.clf()


#"drones_to_depot_packets": [[{"coord": [1918, 193], 

def coverage_distribution(filename_format : str, ndrones : int, 
                        alg_k : int, seeds : list):
    """ plot the coverage distribution 
        return a tuple (X, Y)
    """
    X = []
    Y = []
    for seed in seeds:
        file_name = filename_format.format(ndrones, seed, alg_k)
        with open(file_name, 'r') as fp:
            ktri_0 = json.load(fp)
            delivered_packets = ktri_0["drones_packets"]
            for pack in delivered_packets:
                X.append(pack["coord"][0])
                Y.append(pack["coord"][1])
    return X, Y 

#TODO: this is done for each METRIC!!! can be done once at the beginning for all the metrics
def mean_std_of_metric(filename_format : str, ndrones : int, 
                        alg_k : int, seeds : list, metric : str):
    data = []
    for seed in seeds:
        file_name = filename_format.format(ndrones, seed, alg_k)
        with open(file_name, 'r') as fp:
            ktri_0 = json.load(fp)
            if metric == "ratio_delivery_generated":
                data.append(ktri_0["number_of_events_to_depot"] 
                                        / ktri_0["number_of_generated_events"])
            elif metric == "ratio_delivery_detected":
                data.append(ktri_0["number_of_events_to_depot"] 
                                / ktri_0["number_of_detected_events"])
            elif metric == "Routing time / mission time":
                data.append(ktri_0["time_on_active_routing"]
                                / ktri_0["time_on_mission"])
            elif metric == "energy_move_routing":
                data.append(sum(ktri_0[metric].values()))
            else:
                data.append(ktri_0[metric])

    return np.nanmean(data), np.nanstd(data)


def plot_ndrones(filename_format : list, ndrones :list, metric : str, 
                alg_ritrasmission : list, seeds :list, out_dir : str, 
                exp_metric : str):
    """ plot for varying ndrones """
    
    x = list(ndrones)
    # { k_0 : [y_1, y_2, y_3, .... ]}
    # { k_250 : [y_1, y_2, y_3, .... ]}
    out_data = {} #{ alg_k : [] for alg_k in alg_ritrasmission }
    # for each algortihms (k)
    for alg_k in alg_ritrasmission:
        data_alg_k = []
        #for each x ticks 
        for nd in n_drones:
            data_alg_k.append(mean_std_of_metric(filename_format, nd, alg_k, seeds, metric)[0])
        out_data[alg_k] = data_alg_k
    
    ax = plt.subplot(111)
    fig = plt.gcf() # get current figure
    fig.set_size_inches(16, 10)

    ax.grid()
    for alg_k in out_data.keys():
        y_data = out_data[alg_k]
        #TODO: texture and colors and linestyle for the results 
        ax.plot(x, y_data, label=alg_k)

    plt.ylabel(metric, fontsize=LABEL_SIZE)
    plt.xlabel(exp_metric.replace("_", ""), fontsize=LABEL_SIZE)

    plt.xticks(ndrones)
    
    plt.title(str(alg_ritrasmission) + "_" + metric)
    
    handles, labels = ax.get_legend_handles_labels()
    plt.legend() #handles, labels, prop={'size': LEGEND_SIZE})
    
    if metric == "Routing time / mission time":
        metric ="routing_time_mission_time"
    #plt.tight_layout()
    plt.savefig(out_dir + metric + ".png")
    #plt.show()
    plt.clf()

def set_font():
    font = {'family' : 'normal',
            'size'   : ALL_SIZE}
    matplotlib.rc('font', **font)

METRICS_OF_INTEREST = [
        'number_of_generated_events',
        'number_of_events_to_depot',
        'event_mean_delivery_time',
        'ratio_delivery_detected',
        'Routing time / mission time',
        "energy_move_routing",
        "score"]


if __name__ == "__main__":
    #set matplotlib font 
    set_font()

    parser = ArgumentParser()
    
    routing_choices = config.RoutingAlgorithm.keylist()

    # MANDATORY
    parser.add_argument("-nd", dest='number_of_drones', action="append", type=int,
                        help="the number of drones to use in the simulataion")
    parser.add_argument("-i_s", dest='initial_seed', action="store", type=int,
                        help="the initial seed to use in the simualtions")
    parser.add_argument("-e_s", dest='end_seed', action="store", type=int,
                        help="the end seed to use in the simualtions"   
                             + "-notice that the simulations will run for seed in (i_s, e_s)")
    parser.add_argument("-exp_suffix", dest='alg_exp_suffix', action="append", type=str,
                        help="the routing algorithm suffix to read exp data: es: K_ROUTING_500 or MOVE")
    parser.add_argument("-exp_metric", dest='exp_metric', action="store", type=str, default="ndrones_",
                        help="the exp metric to run, should be in [ninterval_ speed_ ndrones_] ")
     
     

    args = parser.parse_args()

    #drones
    n_drones = args.number_of_drones
    
    #suffix for metric
    exp_metric = args.exp_metric

    #initial seed
    initial_seed = args.initial_seed
    end_seed = args.end_seed
    n_seeds = list(range(initial_seed, end_seed))  #available seed
    
    dim_area = 1500

    #alg suffix 
    alg_exp_suffix = args.alg_exp_suffix
    
    
    pattern_file = config.EXPERIMENTS_DIR + "out__" + exp_metric + "{}_seed{}_alg_{}.json"
    out_dir = config.SAVE_PLOT_DIR
    print(alg_exp_suffix)
    for metric in METRICS_OF_INTEREST:
        plot_ndrones(pattern_file, n_drones, metric, alg_exp_suffix, n_seeds, out_dir + "_" + str(exp_metric) + "_", exp_metric)
