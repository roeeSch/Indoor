#!/usr/bin/env python
import math
import numpy as np

class LocalPlanner:
    def __init__(self, Grid):
        self.Grid = Grid

    def LocalPathFinder(self, current_pos, current_heading, dict_of_drones_pos, agents, agent_idx):
        next_heading = self.RotationAngleManager()
        agents[agent_idx].preform_step_sys_sim(current_pos, current_heading, self.Grid, next_heading)
        dict_of_drones_pos[agent_idx].pos = agents[agent_idx].current_pos[0]
        dict_of_drones_pos[agent_idx].next_pos = agents[agent_idx].next_pos[0]
        dict_of_drones_pos[agent_idx].yaw = next_heading
        if agents[agent_idx].astar_path != []:
            dict_of_drones_pos[agent_idx].goal = agents[agent_idx].astar_path[-1]
        else:
            dict_of_drones_pos[agent_idx].goal = dict_of_drones_pos[agent_idx].next_pos

        return dict_of_drones_pos, agents

    def RotationAngleManager(self):
        return np.random.rand() * np.pi / 4