#!/usr/bin/env python
import numpy as np
from bresenham import bresenham

class WPmonitoring:
    def __init__(self, Prefix, x_lim, y_lim, res, current_pos):
        self.Prefix = Prefix
        self.x_lim = x_lim
        self.y_lim = y_lim
        self.res = res
        self.current_pos = current_pos
        self.next_pos = current_pos
        self.DroneTrj = []
        self.stepSizeLimit = 50
        self.dist_factor = 1
        self.step_noise_size = 20
        self.current_heading = []
        self.next_heading = []


    def FollowWP(self, current_pos, current_heading, Grid, trajectory, next_heading):
        self.current_pos = current_pos
        self.DroneTrj = trajectory
        self.preform_step_sys_sim(current_pos, current_heading, next_heading, Grid)


    def update_current_state(self, current_pos, current_heading):
        self.current_pos = current_pos
        self.current_heading = current_heading


    def preform_step_sys_sim(self, current_pos, current_heading, next_heading, matrix):
        self.update_current_state(current_pos, current_heading)
        self.Dynam_Search_in_maze(matrix)
        self.next_heading = next_heading


    def Dynam_Search_in_maze(self, matrix):

        max_count_val = 15
        break_counter = 0
        vec = np.zeros(2)
        flag = False
        tails_from_wall = 1
        as_flag = False
        close_wall = False

        # If there are steps left in the path and the next step is in line of sight then choose it.
        if self.DroneTrj != [] and self.is_step_legal(self.current_pos,
                                                        np.subtract(self.DroneTrj[0], self.current_pos[0]), matrix):
            vec = np.subtract(self.DroneTrj[0], self.current_pos[0])
            as_flag = True
        # If there are steps left in the path and the previous step is not finished and still legal then
        # resume the previous step
        elif self.DroneTrj != [] and np.linalg.norm(
                np.subtract(self.current_pos[0], self.next_pos[0])) > self.dist_factor * self.step_noise_size \
                and self.is_step_legal(self.current_pos, np.subtract(self.next_pos[0], self.current_pos[0]), matrix):
            vec = np.subtract(self.next_pos[0], self.current_pos[0])
        # If there are no steps in path and the previous step is still legal then resume the previous step
        elif self.is_step_legal(self.current_pos, np.subtract(self.next_pos[0], self.current_pos[0]), matrix):
            vec = np.subtract(self.next_pos[0], self.current_pos[0])

        # Check if the chosen step will be to close to a wall
        if sum(vec) != 0:
            ivec, jvec = self.xy_to_ij(vec[0], vec[1])
            for ti in range(ivec - tails_from_wall, ivec + tails_from_wall + 1):
                for tj in range(jvec - tails_from_wall, jvec - tails_from_wall + 1):
                    if matrix[ti][tj] == 1:
                        close_wall = True
                        break

        # If indeed it is to close to a wall then move in the same direction but stop a few tail before the wall
        if close_wall:
            if np.linalg.norm(np.subtract(self.current_pos[0], self.next_pos[0])) > self.res:
                step = np.multiply(np.divide(vec, np.linalg.norm(vec)),
                                   np.linalg.norm(vec) - (tails_from_wall * self.res))
                if (np.linalg.norm(vec) - (tails_from_wall * self.res)) > 0:
                    vec = step

        # Limit the step size to maximum distance
        if np.linalg.norm(vec) > self.stepSizeLimit:
            temp = np.divide(vec, np.linalg.norm(vec))
            vec = np.multiply(temp, self.stepSizeLimit)

        # Add noise (only for simulation)
        while break_counter < max_count_val:
            break_counter = break_counter + 1
            step = self.step_noise_size * ([0.5, 0.5] - np.random.rand(2)) + vec
            if self.is_step_legal(self.current_pos, step, matrix):
                break

        # Update the step and erase the wp from the path
        if break_counter < max_count_val:
            self.next_pos = self.current_pos + step
            if as_flag and self.DroneTrj != []:
                del self.DroneTrj[0]


    def is_step_legal(self, curr_pos, step, matrix):
        new_pos = curr_pos + step
        i, j = self.xy_to_ij(new_pos[0][0], new_pos[0][1])
        if not (0 <= i and i < matrix.shape[0] and 0 <= j and j < matrix.shape[1]):
            return False
        return self.is_los(curr_pos, new_pos, matrix)


    def xy_to_ij(self, x, y):
        i = int(np.floor((x - self.x_lim[0]) / self.res))
        j = int(np.floor((y - self.y_lim[0]) / self.res))
        return i, j


    def ij_to_xy(self, i, j):
        x = self.x_lim[0] + i * self.res + self.res / 2
        y = self.y_lim[0] + j * self.res + self.res / 2
        return x, y


    # def is_los(self, p1, p2, matrix):
    #     i0, j0 = self.xy_to_ij(p1[0][0], p1[0][1])
    #     i1, j1 = self.xy_to_ij(p2[0][0], p2[0][1])
    #     bres_list = list(bresenham(i0, j0, i1, j1))
    #     for ind in range(len(bres_list)):
    #         i, j = bres_list[ind]
    #         if matrix[i][j] != 0:
    #             return False
    #     return True


    def is_los(self, p1, p2, matrix):
        n = int(np.maximum(1, np.ceil(np.linalg.norm(p1 - p2) / self.res) * 3))
        x = np.linspace(p1[0][0], p2[0][0], num=n, endpoint=True)
        y = np.linspace(p1[0][1], p2[0][1], num=n, endpoint=True)
        for ind in range(1, n):
            i, j = self.xy_to_ij(x[ind], y[ind])
            if matrix[i][j] != 0:
                return False
        return True