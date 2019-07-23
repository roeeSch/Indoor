#!/usr/bin/env python
import numpy as np
import Astar
import copy

class LocalMissionPlanner:
    def __init__(self, env_limits, tf_prefix, res, curpos, grid, curyaw):
        self.env_limits = env_limits
        self.tf_prefix = tf_prefix
        self.res = res
        self.curpos = curpos
        self.nextpos = copy.deepcopy(curpos)
        self.takeoffpos = copy.deepcopy(curpos)
        self.grid = grid
        self.curyaw = curyaw
        self.nextyaw = copy.deepcopy(curyaw)
        self.traj = []

    def TaskAssignment(self, dict_of_drones_pos, dronepos, next_pos, grid, droneyaw, traj):

        # Parameters for local mission planner
        if traj == []:
            nextpos = next_pos
        else:
            nextpos = [traj[0]]

        self.curpos = dronepos
        self.nextpos = nextpos
        self.curyaw = droneyaw
        self.traj = traj
        self.grid = grid
        self.MaintainSearchState(dict_of_drones_pos)
        self.nextyaw = self.curyaw + self.RotationAngleManager()

    def RotationAngleManager(self):
        return np.random.rand() * np.pi / 4

    def MaintainSearchState(self, dict_of_drones_pos):

        Astar_Movement = Astar.build_trj(self.curpos, self.env_limits, self.res, self.grid, self.nextpos,
                                         self.tf_prefix, dict_of_drones_pos)
        if Astar_Movement == []:
            Astar_Movement = [self.nextpos[0]]

        self.traj = Astar_Movement + self.traj

    def ReturnToHome(self, dict_of_drones_pos):

        self.nextpos = self.takeoffpos
        Astar_Movement = Astar.build_trj(self.curpos, self.env_limits, self.res, self.grid, self.nextpos,
                                         self.tf_prefix,
                                         dict_of_drones_pos)
        if Astar_Movement == []:
            Astar_Movement = [self.curpos[0], self.nextpos[0]]

        self.traj = Astar_Movement

    def Land(self):
        pass

    def FindCommunication(self):
        pass

    def ExploreNewArea(self):
        pass

    def ImproveOdometry(self):
        pass


