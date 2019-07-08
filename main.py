# **************** main *******************
import matplotlib.pyplot as plt
import matplotlib.animation as manimation
import numpy as np
import time
from EnvSim import Env
from Grid import Grid
from Agent import Agent
from Drone import Drone
import PathBuilder
from GridPOI import GridPOI
from Display import Display
import copy
import Astar
import CentralpathPlanner
import LocalPlanner

class DronePosGoal:
    """A simple class for holding drone position."""

    def __init__(self, pos=None, next_pos=None, goal=None, yaw=None):
        self.pos = pos
        self.next_pos = next_pos
        self.goal = goal
        self.yaw = yaw


sleep_time = 0.01
num_of_agents = 5
num_of_steps = 5
count_time_step = 0
time_multiplier = 3
num_of_iter = 1000
resolution = 10

env = Env(resolution, 2)
dict_of_drones_pos = dict()
grid = Grid(env.border_polygon_for_grid, resolution)
grid_poi = GridPOI(grid.res, grid.x_lim, grid.y_lim)

agents = list()
drones = list()
drones_pos_list = list()
for i in range(0, num_of_agents):
    drone_pos = [[-10000, -10000]]
    while (env.is_in_obs(drone_pos)) or not(env.is_in_border_polygon(drone_pos)):
        drone_pos = env.enterence + 30 * 2 * ([[0.5, 0.5]] - np.random.rand(1, 2))

    drones.append(Drone(i, drone_pos, 0, env))
    agents.append(Agent(i, drone_pos, grid.res, grid.x_lim, grid.y_lim))
    drones_pos_list.append(list(drone_pos[0]))

display = Display(env.border_polygon, env.obs_array, grid.x_lim, grid.y_lim, grid.res, grid.matrix, drones_pos_list)

for i in range(0, num_of_agents):
    dict_of_drones_pos[i] = DronePosGoal(pos=drones[i].pos[0], next_pos=drones[i].pos[0], goal=drones[i].pos[0], yaw=0)

movie_flag = False
if not movie_flag:
    for t in range(1, num_of_iter):
        count_time_step += 1
        if np.mod(count_time_step, time_multiplier) == 0:

            centralpathplanner = CentralpathPlanner.CentralpathPlanner(grid.matrix, num_of_agents, num_of_steps, grid.x_lim, grid.y_lim, grid.res)
            dict_of_drones_pos = centralpathplanner.BuildPath(dict_of_drones_pos)

            for i in range(0, num_of_agents):

                env_limits = grid.x_lim + grid.y_lim
                goal = dict_of_drones_pos[i].goal
                tf_prefix = i
                Astar_Movement = Astar.build_trj(drones[i].pos, env_limits, grid.res, grid.matrix, goal, tf_prefix, dict_of_drones_pos)

                if Astar_Movement == []:
                    Astar_Movement = [[drones[i].pos[0][0], drones[i].pos[0][1]], goal]
                agents[i].astar_path = Astar_Movement

        for i in range(0, num_of_agents):

            tof_list = drones[i].tof_sensing()
            grid.update_from_tof_sensing_list(tof_list)
            drones[i].preform_step(drones)
            localplanner = LocalPlanner.LocalPlanner(grid.matrix)
            dict_of_drones_pos, agents = localplanner.LocalPathFinder(drones[i].pos, drones[i].current_heading, dict_of_drones_pos, agents, i)
            drones[i].update_virtual_targets(agents[i].next_pos, agents[i].next_heading)
            display.plot_step(agents[i].next_pos, grid.empty_idxs, grid.wall_idxs, drones[i].pos, i)

        display.fig.canvas.draw()
        time.sleep(sleep_time)

else: # Creating a movie
    FFMpegWriter = manimation.writers['ffmpeg']
    metadata = dict(title='Movie Test', artist='Matplotlib',
                    comment='Movie support!')
    writer = FFMpegWriter(fps=10, metadata=metadata)
    with writer.saving(display.fig, "writer_test.mp4", 100):

            writer.grab_frame()