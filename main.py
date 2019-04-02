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


env = Env(10, 2, True)
grid = Grid(env.border_polygon_for_grid, 10, env.ax_grid)
grid.plot_grid()
grid_poi = GridPOI(grid.res, grid.x_lim, grid.y_lim)

env.plot_obs_array()
env.fig.show()
env.fig.canvas.draw()

sleep_time = 0.01
n = 5 # number of agents in the scenario
agents = list()
drones = list()
for i in range(0, n):
    drone_pos = [[-10000, -10000]]
    while (env.is_in_obs(drone_pos)) or not(env.is_in_border_polygon(drone_pos)):
        drone_pos = env.enterence + 30 * 2 * ([[0.5, 0.5]] - np.random.rand(1, 2))

    drones.append(Drone(i, drone_pos, 0, env, env.ax, env.ax_grid))
    agents.append(Agent(i, drone_pos, grid.res, grid.x_lim, grid.y_lim))

env.fig.show()
env.fig.canvas.draw()
time.sleep(sleep_time)

# RL
count_time_step = 0
time_multiplier = 7
num_of_iter = 1000
corner_points_list_ij = []
interesting_points_list_ij = []
#
for i in range(0, n):
    virtual_target, virtual_heading = agents[i].get_virtual_target_and_heading()

movie_flag = False
if not movie_flag:
    for t in range(1, num_of_iter):
        count_time_step += 1
        if np.mod(count_time_step, time_multiplier) == 0:

            grid.erase_corner_points(corner_points_list_ij)
            grid.erase_interesting_points(interesting_points_list_ij)

            interesting_points_list_ij, interesting_points_list_xy, \
            corner_points_list_ij, corner_points_list_xy, \
            wall_idxs_ij, wall_idxs_xy = grid_poi.find_POI(grid.matrix)

            grid.plot_corner_points(corner_points_list_ij)
            grid.plot_interesting_points(interesting_points_list_ij)
            grid.complete_wall_corner(wall_idxs_ij)

            Astar_Movement = PathBuilder.build_trj(drones, corner_points_list_xy, interesting_points_list_xy,
                                                   grid.x_lim, grid.y_lim, grid.matrix, grid.res)
            if Astar_Movement:
                for i in range(0, n):
                    agents[i].astar_path = Astar_Movement[i]
        for i in range(0, n):
            tof_list = drones[i].tof_sensing()
            grid.update_from_tof_sensing_list(tof_list)
            drones[i].prevent_collision(drones)
            neigbours_pos_list = drones[i].preform_step(drones)
            virtual_target, virtual_heading = agents[i].preform_step_sys_sim(drones[i].pos, drones[i].current_heading, neigbours_pos_list, grid.matrix)
            drones[i].update_virtual_targets(virtual_target, virtual_heading)
            drones[i].plot_edges(env.ax_grid, agents[i].reduced_neigbours_pos_list)

        for i in range(0, n):
            drones[i].stop_command = False

        env.fig.canvas.draw()
        time.sleep(sleep_time)

else: # Creating a movie
    FFMpegWriter = manimation.writers['ffmpeg']
    metadata = dict(title='Movie Test', artist='Matplotlib',
                    comment='Movie support!')
    writer = FFMpegWriter(fps=10, metadata=metadata)
    with writer.saving(env.fig, "writer_test.mp4", 100):

            writer.grab_frame()