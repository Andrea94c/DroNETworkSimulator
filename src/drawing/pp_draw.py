from src.drawing import stddraw
from src.entities.uav_entities import Environment
from src.utilities import config, utilities
from collections import defaultdict

#printer the environment 
class PathPlanningDrawer():

    #init the drawer for the path planning 
    def __init__(self, env : Environment, simulator,
                 borders=False, padding=25):
        """ init the path plannind drawer """
        self.width =  env.width
        self.height =  env.height
        self.borders = borders
        self.simulator = simulator
        stddraw.setXscale(0 - padding, self.width + padding)
        stddraw.setYscale(0 - padding, self.height + padding)
        if self.borders:
            self.__borders_plot()

        self.__grid_plot()
        self.keep_indictor = defaultdict(list)  # list of couples (time stamp, drone)

    def __channel_to_depot(self):
        stddraw.setPenColor(c=stddraw.LIGHT_GRAY)
        stddraw.setPenRadius(0.0025)
                    #x1, y1, x2, y2
        stddraw.line(self.width/2, self.height, self.width/2, 0)
        self.__reset_pen()


    def save(self, filename):
        """ save the current plot """
        stddraw.save(filename)

    def __borders_plot(self):
        stddraw.setPenColor(c=stddraw.RED)
        stddraw.setPenRadius(0.0025)
        stddraw.line(0, 0, 0, self.width)
        stddraw.line(0, 0, self.height, 0)
        stddraw.line(0, self.width, self.height, self.width)
        stddraw.line(self.height, 0, self.height, self.width)
        self.__reset_pen()

    def __grid_plot(self):
        for i in range(0, self.width, self.simulator.prob_size_cell):
            stddraw.setPenColor(c=stddraw.GRAY)
            stddraw.setPenRadius(0.0025)
            # x1, y1, x2, y2
            stddraw.line(i, 0, i, self.height)
            self.__reset_pen()
        for j in range(0, self.height, self.simulator.prob_size_cell):
            stddraw.setPenColor(c=stddraw.GRAY)
            stddraw.setPenRadius(0.0025)
            # x1, y1, x2, y2
            stddraw.line(0, j, self.width, j)
            self.__reset_pen()

        for cell, cell_center in utilities.TraversedCells.all_centers(self.width, self.height, self.simulator.prob_size_cell):
            index_cell = int(cell[0])
            pr = self.simulator.cell_prob_map[index_cell][2]
            stddraw.text(cell_center[0], cell_center[1], "pr-c: " + str(round(pr, 4)))

    def __reset_pen(self):
        stddraw.setPenColor(c=stddraw.BLACK)
        stddraw.setPenRadius(0.0055)

    def draw_drone(self, drone, cur_step):
        coords = drone.coords
        if drone.buffer_length() > 0:  # change color when find a packet
            stddraw.setPenColor(c=stddraw.GREEN)
        else:
            stddraw.setPenColor(c=stddraw.BLACK)
        stddraw.setPenRadius(0.0055)
        stddraw.point(coords[0], coords[1])

        self.__draw_drone_info(drone, cur_step)
        if drone.move_routing or drone.come_back_to_mission:
            color = stddraw.RED
        else:
            color = stddraw.BLUE

        self.__draw_communication_range(drone, color=color)
        self.__draw_sensing_range(drone)
        self.__reset_pen()

        if config.IS_SHOW_NEXT_TARGET_VEC:
            self.__draw_next_target(drone.coords, drone.next_target())

    def update(self, rate=1, 
                save=False, show=True,
                filename=None):
        """ update the draw """
        if self.borders:
            self.__borders_plot()

        if config.ENABLE_PROBABILITIES:
            self.__grid_plot()
        if show:
            stddraw.show(rate)
        if save:
            assert(filename is not None)
            self.save(filename)
        stddraw.clear()
        
    def draw_event(self, event):
        coords = event.coords
        stddraw.setPenRadius(0.0055)
        stddraw.setPenColor(c=stddraw.RED)
        stddraw.point(coords[0], coords[1])
        stddraw.setPenColor()
        self.__reset_pen()

    def draw_vector(self, pos, vector):
        stddraw.setPenRadius(0.0500)
        stddraw.line(pos[0], pos[1], 
                        vector[0], vector[1])
    

    def draw_depot(self, depot):
        coords = depot.coords
        stddraw.setPenRadius(0.0100)
        stddraw.setPenColor(c=stddraw.DARK_RED)
        size_depot = 50
        stddraw.filledPolygon([coords[0] - (size_depot / 2), coords[0], coords[0] + (size_depot / 2)],
                        [coords[1], coords[1] + size_depot, coords[1]])
        self.__draw_communication_range(depot)
        self.__reset_pen()

        # draw the buffer size
        stddraw.setPenRadius(0.0125)
        stddraw.setPenColor(c=stddraw.BLACK)
        stddraw.text(depot.coords[0], depot.coords[1]+100, "pk: " + str(len(depot.all_packets())))

    def __draw_sensing_range(self, body):
        stddraw.setPenRadius(0.0015)
        stddraw.setPenColor(c=stddraw.RED)
        stddraw.circle(body.coords[0], body.coords[1], 
                        body.sensing_range)
        stddraw.setPenColor(c=stddraw.BLACK)
        
    def __draw_communication_range(self, body, color=stddraw.BLUE):
        stddraw.setPenRadius(0.0015)

        stddraw.setPenColor(c=color)
        stddraw.circle(body.coords[0], body.coords[1], 
                        body.communication_range)
        stddraw.setPenColor(c=stddraw.BLACK)

    def draw_blocks(self, drone_coo, target, size_cell, cells):

        for coord in cells:
            stddraw.setPenColor(c=stddraw.CYAN)
            stddraw.filledRectangle(coord[0] * size_cell, coord[1] * size_cell, size_cell, size_cell)
            self.__reset_pen()

    def __draw_next_target(self, drone_coo, target):
        stddraw.setPenRadius(0.0055)
        stddraw.setPenColor(c=stddraw.BLUE)
        stddraw.point(target[0], target[1])
        stddraw.setPenColor()
        self.__reset_pen()

        stddraw.setPenColor(c=stddraw.BLUE)
        stddraw.setPenRadius(0.0025)
        stddraw.line(drone_coo[0], drone_coo[1], target[0], target[1])
        self.__reset_pen()

    def __draw_drone_info(self, drone, cur_step):
        stddraw.setPenRadius(0.0125)
        stddraw.setPenColor(c=stddraw.BLACK)
        # life time and speed
        stddraw.text(drone.coords[0]-50, drone.coords[1], "buf: " + str(drone.buffer_length()))
        # life time and speed
        #stddraw.text(drone.coords[0]+75, drone.coords[1], " e:" + str(int(drone.residual_energy)))
        # index
        stddraw.text(drone.coords[0], drone.coords[1] + (drone.communication_range / 2.0), "id: " + str(drone.identifier))

        if drone.buffer_length() > 0:
            stddraw.text(drone.coords[0], drone.coords[1] - (drone.communication_range / 2.0), "retr: " +
                         str(drone.routing_algorithm.current_n_transmission))

        # If the buffer is empty, do not show the retransmission counters since they are not updated
        else:
            stddraw.text(drone.coords[0], drone.coords[1] - (drone.communication_range / 2.0), "retr: -- \\ --")

    def draw_simulation_info(self, cur_step, max_steps):
        TEXT_LEFT = 60
        TEXT_TOP = 20
        stddraw.text(TEXT_LEFT + 20, self.height - TEXT_TOP, str(cur_step) + "/" + str(max_steps))
        self.__reset_pen()