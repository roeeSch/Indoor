#!/usr/bin/env python
import math
import numpy as np

class Guidance:
    def __init__(self, Prefix):
        self.Prefix = Prefix

    def WPmonitoring(self, current_pos, current_heading, dict_of_drones_pos, agents, Grid, next_heading):
        agents[self.Prefix].preform_step_sys_sim(current_pos, current_heading, Grid, next_heading)
        dict_of_drones_pos[self.Prefix].pos = agents[self.Prefix].current_pos[0]
        dict_of_drones_pos[self.Prefix].next_pos = agents[self.Prefix].next_pos[0]
        dict_of_drones_pos[self.Prefix].yaw = next_heading
        if agents[self.Prefix].astar_path != []:
            dict_of_drones_pos[self.Prefix].goal = agents[self.Prefix].astar_path[-1]
        else:
            dict_of_drones_pos[self.Prefix].goal = dict_of_drones_pos[self.Prefix].next_pos

        return dict_of_drones_pos, agents



