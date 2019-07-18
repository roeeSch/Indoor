#!/usr/bin/env python
import math
import numpy as np

class CentralpathPlanner:
    def __init__(self, num_of_agents, num_of_steps, x_lim, y_lim, res):
        self.x_lim = x_lim
        self.y_lim = y_lim
        self.res = res
        self.num_of_agents = num_of_agents
        self.num_of_steps = num_of_steps
        self.x_grid = np.round((x_lim[1] - x_lim[0]) / 8)
        self.y_grid = np.round((y_lim[1] - y_lim[0]) / 8)


    def BuildPath(self, dict_of_drones_pos, Grid):

        for k in range(self.num_of_agents):

            current_drone_pos = dict_of_drones_pos[k].pos[0]
            x_min = np.maximum(current_drone_pos[0] - self.x_grid, self.x_lim[0])
            x_max = np.minimum(current_drone_pos[0] + self.x_grid, self.x_lim[1])
            y_min = np.maximum(current_drone_pos[1] - self.y_grid, self.y_lim[0])
            y_max = np.minimum(current_drone_pos[1] + self.y_grid, self.y_lim[1])

            i_min, j_min = self.xy_to_ij(x_min, y_min)
            i_max, j_max = self.xy_to_ij(x_max, y_max)
            empty_tails = []
            for i in range(i_min, i_max):
                for j in range(j_min, j_max):
                    if Grid[i][j] == 0:
                        empty_tails.append([i, j])

            g_idx = np.random.randint(len(empty_tails))
            goal = empty_tails[g_idx]
            dict_of_drones_pos[k].trajectory = [np.array(self.ij_to_xy(goal[0], goal[1]))]

        return dict_of_drones_pos


    def FindMultiPath(self):
        pass

    def EvaluatePath(self):
        pass

    def EvaluatePath(self):
        pass

    def xy_to_ij(self, x, y):
        i = int(np.floor((x - self.x_lim[0])/self.res))
        j = int(np.floor((y - self.y_lim[0]) / self.res))
        return i, j

    def ij_to_xy(self, i, j):
        x = self.x_lim[0] + i*self.res + self.res/2
        y = self.y_lim[0] + j*self.res + self.res/2
        return x, y

