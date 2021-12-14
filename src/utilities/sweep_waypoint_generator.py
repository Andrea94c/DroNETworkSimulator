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


"""
    autonomy is the total time
"""
def get_sweepcoverage_tour(autonomy, edge_area, ndrones, sensing_radius, random_generator, fixed_starting_point=False):
    # Compute the area window size of a drone tour
    window_size = int(math.sqrt((edge_area * edge_area) / min(ndrones, 10)))

    if fixed_starting_point:
        fixed_starting_points = [(random_generator.randint(0,25), random_generator.randint(0,25)),
                                 (random_generator.randint(980, 1250), random_generator.randint(0,25)),
                                 (random_generator.randint(0,25), random_generator.randint(980, 1250)),
                                 (random_generator.randint(980, 1250), random_generator.randint(980, 1250))]
        start_point = fixed_starting_points[random_generator.randint(0, len(fixed_starting_points))]
    else:
        start_point = (random_generator.randint(0, edge_area), random_generator.randint(0, edge_area))
    end_point = (start_point[0] + window_size, start_point[1] + window_size)

    # until the window is totally or partially out of the mission area recalculate it
    while (end_point[0] >= edge_area or end_point[1] >= edge_area):
        start_point = (random_generator.randint(0, edge_area), random_generator.randint(0, edge_area))
        end_point = (start_point[0] + window_size, start_point[1] + window_size)

    # Initial condition. (depot position)
    current_point = start_point

    stepSizeX = window_size
    stepSizeY = sensing_radius

    x = start_point[0]
    y = start_point[1]

    # 1 is to the right 0 si to the left
    required_motion = 1

    # if 1 means that the sweep coverage will have long lines on y respect to x
    direction = random_generator.randint(0, 1)

    graph_tour = []
    drone_tour = []

    # If the new step is calculable inside the mission window
    while (y + stepSizeY < end_point[1]):

        # if the new x is not escaping its window size to the right
        if (x + stepSizeX <= end_point[0] and required_motion == 1):
            x = x + stepSizeX
            y_angle = y + stepSizeY
            # in the next step the motion should be reversed so:
            required_motion = 0

        # if the new x is not escaping its window size to the left
        elif (x - stepSizeX >= start_point[0] and required_motion == 0):
            x = x - stepSizeX
            y_angle = y + stepSizeY
            # in the next step the motion should be reversed so:
            required_motion = 1

        # if the direction is 1, the calculated path is 90° rotated
        if (direction == 1):
            # add the new edges
            # the edge before the sweep path angle
            next_p = (y, x)
            # the edge with the angle
            p_angle = (y_angle, x)
            current_point = (current_point[1], current_point[0])
        # Otherwise the path will be normal
        else:
            # add the new edges
            # the edge before the sweep path angle
            next_p = (x, y)
            # the edge with the angle
            p_angle = (x, y_angle)

        # saving the new segments of the path in the graph
        edge1 = (current_point, next_p)
        edge2 = (next_p, p_angle)
        graph_tour.append(edge1)
        graph_tour.append(edge2)

        # saving the waypoint in the waypoints list for writing a JSON
        drone_tour.append(current_point)
        drone_tour.append(next_p)

        # the last point calculated will be the current one
        current_point = (x, y_angle)

        # Now the last y is the one of the path angle
        y = y_angle

    if (direction == 1):
        # saving the last current point calculated in the waypoint list
        drone_tour.append((current_point[1], current_point[0]))
    else:
        # saving the last current point calculated in the waypoint list
        drone_tour.append(current_point)

    return graph_tour, drone_tour


def plot_tour(tours):
    for d in tours.keys():
        print("\nDrone", d)
        x = []
        y = []
        for t in tours[d]:
            print("len tour", len(t))
            for e in t:
                x.extend([e[0][0], e[1][0]])
                y.extend([e[0][1], e[1][1]])

        plt.plot(x, y, marker='o', label=str(d))

    plt.xticks([])
    plt.yticks([])

    plt.xlim(-50, 1550)
    plt.ylim(-50, 1550)
    # plt.legend()
    #plt.show()


def get_tour(autonomy, edge_area, depot_pos, random_generator, ndrones, sensing_range, random_starting_point=True, plot=False):
    """ random_starting_point : whether start the mission from a random point (True) or from the depot (False)"""
    graph, tour = get_sweepcoverage_tour(autonomy, edge_area, ndrones, 100, random_generator)

    if plot:
        out_dict = {1 : [graph]}
        plot_tour(out_dict)

    if random_starting_point:
        return tour
    else:  #append start from depot
        return [depot_pos] + tour