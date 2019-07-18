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
from CentralpathPlanner import CentralpathPlanner
from Guidance import Guidance
from LocalMissionPlanner import LocalMissionPlanner

class DronePosGoal:
    """A simple class for holding drone position."""

    def __init__(self, pos=[], next_pos=[], trajectory=[], yaw=[]):
        self.pos = pos
        self.next_pos = next_pos
        self.trajectory = trajectory
        self.yaw = yaw

sleep_time = 0.01
num_of_agents = 5
num_of_steps = 5
count_time_step = 0
time_multiplier = 5
num_of_iter = 1000
resolution = 10

env = Env(resolution, 2)
dict_of_drones_pos = dict()
grid = Grid(env.border_polygon_for_grid, resolution)
env_limits = grid.x_lim + grid.y_lim

agents = list()
drones = list()
localmissionplanner = list()
guidance = list()
drones_pos_list = list()
for i in range(0, num_of_agents):
    drone_pos = [[-10000, -10000]]
    while (env.is_in_obs(drone_pos)) or not(env.is_in_border_polygon(drone_pos)):
        drone_pos = env.enterence + 30 * 2 * ([[0.5, 0.5]] - np.random.rand(1, 2))

    drones.append(Drone(i, drone_pos, 0, env))
    localmissionplanner.append(LocalMissionPlanner(env_limits, i, grid.res, drone_pos))
    # agents.append(Agent(i, drone_pos, grid.res, grid.x_lim, grid.y_lim))
    drones_pos_list.append(list(drone_pos[0]))
    guidance.append(Guidance(i, grid.x_lim, grid.y_lim, grid.res, drone_pos))

display = Display(env.border_polygon, env.obs_array, grid.x_lim, grid.y_lim, grid.res, grid.matrix, drones_pos_list)
centralpathplanner = CentralpathPlanner(num_of_agents, num_of_steps, grid.x_lim, grid.y_lim, grid.res)

for i in range(0, num_of_agents):
    dict_of_drones_pos[i] = DronePosGoal(pos=drones[i].pos, next_pos=drones[i].pos, trajectory=[], yaw=0)

movie_flag = False
if not movie_flag:
    for t in range(1, num_of_iter):
        count_time_step += 1
        if np.mod(count_time_step, time_multiplier) == 0:
            dict_of_drones_pos = centralpathplanner.BuildPath(dict_of_drones_pos, grid.matrix)

        for i in range(0, num_of_agents):

            tof_list = drones[i].tof_sensing()
            grid.update_from_tof_sensing_list(tof_list)
            drones[i].preform_step(drones)
            dict_of_drones_pos = localmissionplanner[i].TaskAssignment(dict_of_drones_pos, drones[i].pos, grid.matrix)
            dict_of_drones_pos = guidance[i].WPmonitoring(drones[i].pos, drones[i].current_heading, dict_of_drones_pos, grid.matrix)
            drones[i].update_virtual_targets(guidance[i].next_pos, guidance[i].next_heading)
            display.plot_step(guidance[i].next_pos, grid.empty_idxs, grid.wall_idxs, drones[i].pos, i)

        display.fig.canvas.draw()
        time.sleep(sleep_time)

else: # Creating a movie
    FFMpegWriter = manimation.writers['ffmpeg']
    metadata = dict(title='Movie Test', artist='Matplotlib',
                    comment='Movie support!')
    writer = FFMpegWriter(fps=10, metadata=metadata)
    with writer.saving(display.fig, "writer_test.mp4", 100):

            writer.grab_frame()