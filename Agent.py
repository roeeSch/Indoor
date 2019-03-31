# -*- coding: utf-8 -*-
"""
Created on Fri Oct  5 08:59:56 2018

@author: manorr
"""
import matplotlib.pyplot as plt
import matplotlib.animation as manimation
import numpy as np
import time
from EnvSim import Env
from Grid import Grid


class Agent:

    def __init__(self, AgentID, pos, grid, ax, ax_grid):
        self.ID = AgentID
        self.agent_alive = True
        self.is_homing = False
        self.velocityFactor = 50
        self.step_noise_size = 20
        self.step_snr = 1
        self.stepSizeLimit = 30
        self.step_factor = 1
        self.next_pos = pos
        self.current_pos = self.next_pos
        self.next_heading = 0
        self.current_heading = self.next_heading
        self.VisibilityRange = 300.
        self.scanning_range = 200
        self.repulse_range = self.VisibilityRange/10
        self.pull_range = self.VisibilityRange*4/5
        self.goal_orianted_flag_flip_prob = 0
        self.goal_orianted_flag = True #np.random.rand(1) < self.goal_orianted_flag_flip_prob
        self.reduced_neigbours_pos_list = list()
        self.astar_path = [] #RL

        self.grid = grid
        self.ax = ax
        self.plot_color = 'ob'
        # self.AgetPlotHadel, = ax.plot(self.current_pos[0][0], self.current_pos[0][1], self.plot_color)
        # self.AgetPlotHadel_grid, = ax_grid.plot(self.current_pos[0][0], self.current_pos[0][1], self.plot_color)
        self.edg_to_neighbors_plot_hadels = list()

    def update_current_state(self, current_pos, current_heading):
        self.current_pos = current_pos
        self.current_heading = current_heading

    def get_virtual_target_and_heading(self):
        return self.next_pos, self.next_heading

    def preform_step_sys_sim(self, current_pos, current_heading, neigbours_pos_list):
        self.update_current_state(current_pos, current_heading)
        self.reduced_neigbours_pos_list = self.neighborhood_reduction(neigbours_pos_list, self.grid)
        self.Dynam_Search_in_maze(self.reduced_neigbours_pos_list, self.grid)  # This function should be replaced - Rita
        self.next_heading = np.random.rand() * np.pi / 4
        # self.delete_edges()
        # self.plot_edges(self.ax, neigbours_pos_list)

        return self.get_virtual_target_and_heading()

    def calculate_step(self, agentsArr, env):
        self.delete_edges()
        self.plot_agent()
        neigbours_pos_list = self.Sensing(agentsArr, env)
        neigbours_pos_list = self.neighborhood_reduction(neigbours_pos_list, env)
        self.plot_edges(self.ax, neigbours_pos_list)
        if env.is_detect_trg(self.current_pos) and not env.is_target_mang:
            self.is_homing = True
            env.is_target_mang = True
        if self.is_homing:
            self.dynam_homing(neigbours_pos_list, env)
            self.AgetPlotHadel._color = 'r'
            env.plot_homing(self.current_pos, self.velocityFactor/2)
        else:
            self.Dynam_Search_in_maze(neigbours_pos_list, env)   # This function should be replaced - Rita
        self.next_heading = np.random.rand() * np.pi/4

    def perform_step(self, env):
        self.current_pos = self.next_pos
        self.current_heading = self.next_heading
        directions_vec = np.add([0, np.pi/2, np.pi, 3*np.pi/2], self.current_heading)
        for phi in directions_vec:
            flag, sense_pos = env.TOF_sensing(self.current_pos, phi)
            self.grid.update_with_tof_sensor(self.current_pos, sense_pos, flag)

    def dynam_homing(self, NeighborsPosList, env):
        dist_to_target = np.linalg.norm(env.target_pos - self.current_pos)
        if dist_to_target > self.velocityFactor/2:
            heading_direction = (env.target_pos - self.current_pos)/dist_to_target
            step = heading_direction * self.velocityFactor
            break_counter = 0
            flag = False
            while not flag and break_counter < 20:
                step = step/2
                break_counter = break_counter + 1
                if self.grid.is_step_legal(self.current_pos, step):
                    flag = True
                    for neighbor_pos in NeighborsPosList:
                        if self.outOfLimit_Ando(neighbor_pos, step):
                            flag = False
                            break
                        if not self.is_step_in_corridor(step, neighbor_pos):
                            flag = False
                            break

            if break_counter < 20:
                self.next_pos = self.current_pos + step  # todo: intetegrate VelocityFactor



            self.next_pos = self.current_pos + step
        else:
            self.delete_edges()
            self.agent_alive = False


    def Dynam_Search_in_maze(self, NeighborsPosList, grid): # RL

        max_count_val = 15
        break_counter = 0
        vec = np.zeros(2)
        flag = False

        try:
            vec = [self.astar_path[0][0]-self.current_pos[0][0],self.astar_path[0][1]-self.current_pos[0][1]]
        except:
            vec = [self.next_pos[0][0] - self.current_pos[0][0], self.next_pos[0][1] - self.current_pos[0][1]]
        while not flag and break_counter < max_count_val:
            break_counter = break_counter + 1
            step = self.step_noise_size * ([0.5, 0.5] - np.random.rand(2)) + vec
            if grid.is_step_legal(self.current_pos, step):
                flag = True
                if np.random.rand(1) < 0.8:
                    for neighbor_pos in NeighborsPosList:
                        if self.outOfLimit_Ando(neighbor_pos, step):
                            flag = False
                            break
                        if not self.is_step_in_corridor(step, neighbor_pos, grid):
                            flag = False
                            break
                    else:
                        break

        if break_counter < max_count_val:
            self.next_pos = self.current_pos + step
            try:
                del self.astar_path[0]
            except:
                pass


# This is the important function, that should be rewriten
    def Dynam_Search_in_maze_old(self, NeighborsPosList, env):
        flag = False
        break_counter = 0
        rep_att_vec = np.zeros(2)

        # Goal oriented movement
        noise_fac = 1
        if self.goal_orianted_flag:
            optinal_goal_list = self.grid.non_scaned_list(self.current_pos, 5 * self.scanning_range, env)
            if optinal_goal_list.__len__() > 0:
                vec = optinal_goal_list[0] - self.current_pos
                goal_vec = vec / np.linalg.norm(vec)
                rep_att_vec = rep_att_vec + goal_vec
            else:
                noise_fac = 5
        else:
            noise_fac = 2
        # Neighbours oriented movement
        for NeighborPos in NeighborsPosList:
            #rep_att_vec = rep_att_vec - NeighborPos / np.linalg.norm(NeighborPos)
            if np.linalg.norm(NeighborPos) < self.repulse_range:
                rep_att_vec = rep_att_vec - NeighborPos / np.linalg.norm(NeighborPos) * self.VisibilityRange
            if np.linalg.norm(NeighborPos) > self.pull_range*noise_fac:
                rep_att_vec = rep_att_vec + NeighborPos / np.linalg.norm(NeighborPos) * self.VisibilityRange
        if np.linalg.norm(rep_att_vec) > 0:
            rep_att_vec = rep_att_vec/np.linalg.norm(rep_att_vec)
        if np.random.rand() < self.goal_orianted_flag_flip_prob:
            self.goal_orianted_flag = not self.goal_orianted_flag

        if np.linalg.norm(rep_att_vec) > 0:
            rep_att_vec = self.step_noise_size*self.step_snr*rep_att_vec/np.linalg.norm(rep_att_vec)


        while not flag and break_counter < 20:
            break_counter = break_counter + 1
            step = self.step_noise_size * noise_fac * ([0.5, 0.5] - np.random.rand(2)) + rep_att_vec
            if self.grid.is_step_legal(self.current_pos, step):
                flag = True
                for neighbor_pos in NeighborsPosList:
                    if self.outOfLimit_Ando(neighbor_pos, step):
                        flag = False
                        break
                    if not self.is_step_in_corridor(step, neighbor_pos):
                        flag = False
                        break

        if break_counter < 20:
            self.next_pos = self.current_pos + step
    # The "is_step_in_corridor" functions let connected agent to disconnect with small probability
    def is_step_in_corridor(self, step, neighbor_pos, grid):
        neighbor_abs_pos = self.current_pos + neighbor_pos
        if grid.is_step_legal(neighbor_abs_pos, step):
            neighbor_abs_pos_potential = neighbor_abs_pos + step
        else:
            neighbor_pos_unit = neighbor_pos / np.linalg.norm(neighbor_pos)
            neighbor_step_potential = step - 2 * np.dot(step[0], neighbor_pos_unit[0]) * neighbor_pos_unit
            neighbor_abs_pos_potential = neighbor_abs_pos + neighbor_step_potential

        return self.grid.is_los(self.current_pos + step, neighbor_abs_pos_potential)

    def is_step_in_corridor2(self, step, neighbor_pos):
        neighbor_abs_pos = self.current_pos + neighbor_pos
        if self.grid.is_step_legal(neighbor_abs_pos, step):
            neighbor_abs_pos_potential = neighbor_abs_pos + step
            if not self.grid.is_los(self.current_pos + step, neighbor_abs_pos_potential):
                return False

        neighbor_pos_unit = neighbor_pos / np.linalg.norm(neighbor_pos)
        neighbor_step_potential = step - 2 * np.dot(step[0], neighbor_pos_unit[0]) * neighbor_pos_unit
        neighbor_abs_pos_potential = neighbor_abs_pos + neighbor_step_potential
        if self.grid.is_step_legal(neighbor_abs_pos, neighbor_step_potential):
            return self.grid.is_los(self.current_pos + step, neighbor_abs_pos_potential)
        return False

    def outOfLimit_Ando(self, neighbor_pos, step):
        avg_pos = np.divide(neighbor_pos, 2)
        deltas_step = step - avg_pos
        return np.linalg.norm(deltas_step) > self.VisibilityRange/2

    def Sensing(self, agents_arr, env_in):
        neighbors_pos = []
        for i in range(0, agents_arr.__len__()):
            diff = agents_arr[i].current_pos - self.current_pos
            if (agents_arr[i].ID != self.ID and np.linalg.norm(diff) < self.VisibilityRange and
                    env_in.is_los(agents_arr[i].current_pos, self.current_pos)): #TODO: los in the grid or in env?
                neighbors_pos.append(diff)
        return neighbors_pos

    def neighborhood_reduction(self, neighbors_pos, env):
        reduced_neighbors_pos = []
        for i in range(0, neighbors_pos.__len__()):
            flag = True
            counter = 0
            for j in range(0, neighbors_pos.__len__()):
                if i != j and ((np.linalg.norm(neighbors_pos[i]) > np.linalg.norm(neighbors_pos[j])) and
                               (np.linalg.norm(neighbors_pos[i]) >
                                np.linalg.norm(neighbors_pos[j] - neighbors_pos[i]))):
                    if env.is_los(self.current_pos + neighbors_pos[i], self.current_pos + neighbors_pos[j]): #TODO: los in the grid or in env?
                    #    flag = False
                    #    break
                        counter = counter + 1
                        if counter > 0:
                            flag = False
                            break
            if flag:
                reduced_neighbors_pos.append(neighbors_pos[i])
        return reduced_neighbors_pos

    def plot_agent(self):
        if self.goal_orianted_flag:
            col = 'r'
        else:
            col = 'b'
        self.AgetPlotHadel.set_data(self.current_pos[0][0], self.current_pos[0][1])
        self.AgetPlotHadel._color = col
        self.AgetPlotHadel_grid.set_data(self.current_pos[0][0], self.current_pos[0][1])
        self.AgetPlotHadel_grid._color = col

    def plot_edges(self, ax, neighbors_list):
        for pos in neighbors_list:
            edge = ax.plot([self.current_pos[0][0], self.current_pos[0][0]+pos[0][0]], [self.current_pos[0][1], self.current_pos[0][1]+pos[0][1]])
            self.edg_to_neighbors_plot_hadels.append(edge)

    def delete_edges(self):
        for edge_handel in self.edg_to_neighbors_plot_hadels:
            edge_handel[0].remove()
        # self.edg_to_neighbors_plot_hadels.clear()
        del self.edg_to_neighbors_plot_hadels[:]

    def update_sate(self, pos, heading):
        self.current_pos = pos
        self.current_heading = heading

# These functions are not relevant
    def Dynam_los_corridor(self, neighbor_rel_pos, env):
        nieghbor_vecneighbor_unit = neighbor_rel_pos / np.linalg.norm(neighbor_rel_pos)
        left_list = []
        right_list = []
        min_left_cross_prod = self.VisibilityRange/2
        min_right_cross_prod = -self.VisibilityRange / 2
        for vertex in env.obs_vertex_list:
            vec = [vertex[0] - self.Pos[0][0], vertex[1] - self.Pos[0][1]]
            if np.linalg.norm(neighbor_rel_pos/2 - vec) < self.VisibilityRange/2:
                cross_prod = np.cross(nieghbor_vecneighbor_unit, vec)
                if cross_prod > 0:
                    if cross_prod < min_left_cross_prod:
                        min_left_cross_prod = cross_prod
                else:
                    if cross_prod > min_right_cross_prod:
                        min_right_cross_prod = cross_prod
        return min_right_cross_prod, min_left_cross_prod

    def is_in_los_corridor(self, neighbor_rel_pos, env, step):
        min_right_cross_prod, min_left_cross_prod = self.Dynam_los_corridor(neighbor_rel_pos, env)

        nieghbor_vecneighbor = neighbor_rel_pos[0]
        nieghbor_vecneighbor_unit = nieghbor_vecneighbor / np.linalg.norm(nieghbor_vecneighbor)
        cross_prod = np.cross(step, nieghbor_vecneighbor_unit)
        return (min_right_cross_prod < cross_prod) and (cross_prod < min_left_cross_prod)

    def getKey(self, item):
        return item[0]
