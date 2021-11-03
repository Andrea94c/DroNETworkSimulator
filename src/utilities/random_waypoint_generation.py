import json
import random
import math
import matplotlib.pyplot as plt
import numpy as np
from src.utilities import config


## -----------------------------------------------------------------------------
#   _____   _   _  _____        __          __   __     __ _____    ____  _____  _   _  _______ 
#  |  __ \ | \ | ||  __ \       \ \        / //\ \ \   / /|  __ \  / __ \|_   _|| \ | ||__   __|
#  | |__) ||  \| || |  | | ______\ \  /\  / //  \ \ \_/ / | |__) || |  | | | |  |  \| |   | |   
#  |  _  / | . ` || |  | ||______|\ \/  \/ // /\ \ \   /  |  ___/ | |  | | | |  | . ` |   | |   
#  | | \ \ | |\  || |__| |         \  /\  // ____ \ | |   | |     | |__| |_| |_ | |\  |   | |   
#  |_|  \_\|_| \_||_____/           \/  \//_/    \_\|_|   |_|      \____/|_____||_| \_|   |_|   
# 
## -----------------------------------------------------------------------------                                                                                            

def euclidean_distance(point1, point2):
    return math.sqrt(math.pow(point2[0] - point1[0], 2)
                     + math.pow(point2[1] - point1[1], 2))


def next_target(depot_pos, cur_position, residual_autonomy, edge_area, range_decision, random_generator):
    """ return the next position (x,y) along the next autonomy after reached the point"""
    if residual_autonomy < min(range_decision) * 1.44 + euclidean_distance(depot_pos, cur_position):
        return depot_pos, max(0, residual_autonomy - euclidean_distance(cur_position, depot_pos))
    else:
        feasible_positions = [d for d in range_decision if
                              d * 1.44 * 2 + euclidean_distance(cur_position, depot_pos) <= residual_autonomy]

        if len(feasible_positions) == 0:
            return depot_pos, max(0, residual_autonomy - euclidean_distance(cur_position, depot_pos))

        d = random_generator.choice(feasible_positions)

        next_point_x = random_generator.randint(max(0, cur_position[0] - d), min(cur_position[0] + d, edge_area))
        next_point_y = random_generator.randint(max(0, cur_position[1] - d), min(cur_position[1] + d, edge_area))

        next_p = (next_point_x, next_point_y)
        residual_aut = residual_autonomy - euclidean_distance(cur_position, next_p)
        return next_p, residual_aut

def get_tour(autonomy, edge_area, depot_pos, random_generator, index=None, range_decision=None, random_starting_point=True):
    """ random_starting_point : whether start the mission from a random point (True) or from the depot (False)"""
    if range_decision is None:
        range_decision = config.RANDOM_STEPS

    tour = []
    if random_starting_point:
        start_point = (random_generator.randint(0, edge_area), random_generator.randint(0, edge_area))
    else:
        start_point = depot_pos


    if index is not None and index < config.FERRY:
        if index == 1:
            ferry_point = (random_generator.randint(0, edge_area // 2), random_generator.randint(edge_area//2, edge_area))
        elif index == 2:
            ferry_point = (random_generator.randint(edge_area // 3, 2 * edge_area // 3), random_generator.randint(edge_area // 2, edge_area))
        else:
            ferry_point = (random_generator.randint(edge_area // 2, edge_area), random_generator.randint(edge_area // 2, edge_area))

        autonomy = config.DRONE_MAX_ENERGY

    current_point = start_point
    residual_aut = autonomy
    while residual_aut >= min(range_decision) * 1.44 + euclidean_distance(current_point, start_point):

        if index is not None and index < config.FERRY:
            next_p = ferry_point
            if current_point == ferry_point:
                next_p = depot_pos
            residual_aut -= euclidean_distance(current_point, next_p)
        else:
            next_p, residual_aut = next_target(depot_pos, current_point, residual_aut, edge_area, range_decision, random_generator)

            if next_p == depot_pos:
                break
        tour.append(current_point)
        current_point = next_p

    if current_point != depot_pos:  # assert last point is the depot (closed tour)
        tour.append(depot_pos)
    return tour


def random_waypoint_tour(ndrones, nrounds, depot, autonomy, edge_area, random_generator):
    drones_tours = {}
    for d in range(ndrones):
        print(d)
        d_tours = []
        for r in range(nrounds):
            d_tours.append(get_tour(autonomy, edge_area, depot, random_generator=random_generator))
        drones_tours[d] = d_tours
    return drones_tours


## -----------------------------------------------------------------------------
#  _____   _       ____  _______             _____      __      __ ______ 
# |  __ \ | |     / __ \|__   __|   ___     / ____|   /\\ \    / /|  ____|
# | |__) || |    | |  | |  | |     ( _ )   | (___    /  \\ \  / / | |__   
# |  ___/ | |    | |  | |  | |     / _ \/\  \___ \  / /\ \\ \/ /  |  __|  
# | |     | |____| |__| |  | |    | (_>  <  ____) |/ ____ \\  /   | |____ 
# |_|     |______|\____/   |_|     \___/\/ |_____//_/    \_\\/    |______|
#                                                                                                                                                 
## -----------------------------------------------------------------------------

def plot_tour(tours):
    pass

def to_json(tours, mission_data, seed):
    """ take in input the multiround solution
        and print it to json 
    """
    points = set()
    _tours = []
    for d in tours.keys():
        d_tour = []
        for t in tours[d]:
            d_tour.append(t)
            points.add(t)
        _tours.append(d_tour)

    out_json = {"info_mission": mission_data}
    out_json["points"] = [str(p) for p in points]

    out_json["drones"] = []
    for i in range(len(_tours)):
        str_tour = []
        for e in _tours[i]:
            str_tour.append(str(e))
        str_tour.append(str(_tours[i][-1]))
        out_json["drones"].append({
            "index": str(i),
            "tour": str_tour
        })

    with open('data/tours/RANDOM_missions' + str(seed) + '.json', 'w') as outfile:
        json.dump(out_json, outfile)


# -----------------------------------------------------------------------------
#                    __  __            _____  _   _ 
#                   |  \/  |    /\    |_   _|| \ | |
#                   | \  / |   /  \     | |  |  \| |
#                   | |\/| |  / /\ \    | |  | . ` |
#                   | |  | | / ____ \  _| |_ | |\  |
#                   |_|  |_|/_/    \_\|_____||_| \_|
#                                                 
## -----------------------------------------------------------------------------
def run(ndrones, nrounds, autonomy,
        depot, mission_data, edge_area, seed,
        plot=True, save=False):
    print("Number of drones/depots:", ndrones)
    print("Autonomy:", autonomy)
    print("Max number of autonomy:", nrounds)

    # set seed
    random_generator = np.random.RandomState(seed)

    # get tours
    tours = random_waypoint_tour(ndrones, nrounds, depot, autonomy, edge_area, random_generator=random_generator)

    if plot:
        plot_tour(tours)

    if save:
        to_json(tours, mission_data, seed)


"build the json file tours for the routing "
if __name__ == "__main__":

    for seed in range(0, 50):
        # mission info
        depot = (750, 0)
        nrounds = 1
        edge_area = 1500
        aut = 100000  # meters in our simulation use at least 60000
        ndrones = 90
        mission_data = {
            "ndrones": str(ndrones),
            "autonomy_meters": str(aut),
            "edge_area": str(edge_area)
        }
        # save = True -> create the json file with the path
        run(ndrones, nrounds, aut, depot, mission_data, edge_area, seed, plot=False, save=True)
