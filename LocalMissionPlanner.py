import numpy as np
import Astar

class LocalMissionPlanner:
    def __init__(self, env_limits, tf_prefix, res, curpos):
        self.env_limits = env_limits
        self.tf_prefix = tf_prefix
        self.res = res
        self.curpos = curpos
        self.nextpos = curpos

    def TaskAssignment(self, dict_of_drones_pos, dronepos, grid):
        self.curpos = dronepos
        if dict_of_drones_pos[self.tf_prefix].trajectory == []:
            self.nextpos = dict_of_drones_pos[self.tf_prefix].next_pos
        else:
            self.nextpos = [dict_of_drones_pos[self.tf_prefix].trajectory[0]]

        Astar_Movement = Astar.build_trj(self.curpos, self.env_limits, self.res, grid, self.nextpos, self.tf_prefix,
                                         dict_of_drones_pos)
        if Astar_Movement == []:
            Astar_Movement = [self.curpos[0], self.nextpos[0]]

        dict_of_drones_pos[self.tf_prefix].trajectory = Astar_Movement[1:] + dict_of_drones_pos[self.tf_prefix].trajectory
        dict_of_drones_pos[self.tf_prefix].yaw =self.RotationAngleManager()

        return  dict_of_drones_pos

    def RotationAngleManager(self):
        return np.random.rand() * np.pi / 4


