import numpy as np
import Astar

class LocalMissionPlanner:
    def __init__(self, env_limits, tf_prefix, res, curpos):
        self.env_limits = env_limits
        self.tf_prefix = tf_prefix
        self.res = res
        self.curpos = curpos
        self.nextpos = curpos
        self.LMPpath = []

    def TaskAssignment(self, dict_of_drones_pos, dronepos, grid):
        self.curpos = dronepos[0]
        self.nextpos = dict_of_drones_pos[self.tf_prefix].goal
        Astar_Movement = Astar.build_trj(self.curpos, self.env_limits, self.res, grid, self.nextpos, self.tf_prefix,
                                         dict_of_drones_pos)
        if Astar_Movement == []:
            Astar_Movement = [[self.curpos[0], self.curpos[1]], self.nextpos]

        self.LMPpath = Astar_Movement
        next_heading = self.RotationAngleManager()

        return  Astar_Movement, next_heading

    def RotationAngleManager(self):
        return np.random.rand() * np.pi / 4


