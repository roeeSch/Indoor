import matplotlib.pyplot as plt
import numpy as np
import EnvSim

class Drone:
    def __init__(self, AgentID, pos, heading_ang, env, ax_env, ax_grid):
        self.ID = AgentID
        self.env = env
        self.is_alive = True
        self.acc_factor = 10
        self.ang_vel_fac = 10
        self.acc_lim = 100 #[cm^2/sec]
        self.vel_lim = 100 #[cd/sec]
        self.ang_vel_lim = np.pi #[rad/sec]
        self.step_noise_size = 20
        self.stepSizeLimit = 30
        self.virtual_target = pos
        self.pos = pos
        self.virtual_heading = heading_ang
        self.current_heading = heading_ang
        self.vel = [0, 0]   #[cm/sec X 2]
        self.ang_vel = 0    #[cd/sec]
        self.scanning_range = 200 #[cm]
        self.neighbours_visibility_range = 400 #cm
        self.dt = 0.1 #[sec]
        self.stop_command = False #RL

        self.ax_env = ax_env
        self.ax_grid = ax_grid
        self.plot_color = 'ob'
        self.plot_hadel_env, = ax_env.plot(self.pos[0][0], self.pos[0][1], 'ob')
        self.plot_hadel_grid, = ax_grid.plot(self.pos[0][0], self.pos[0][1], 'ob')
        self.plot_hadel_env_vt, = ax_env.plot(self.pos[0][0], self.pos[0][1], 'or')
        self.plot_hadel_grid_vt, = ax_grid.plot(self.pos[0][0], self.pos[0][1], 'or')
        self.plot_hadel_env_l2vt, = ax_env.plot([self.pos[0][0], self.pos[0][0]], [self.pos[0][1], self.pos[0][1]], '--r')
        self.plot_hadel_grid_l2vt, = ax_grid.plot([self.pos[0][0], self.pos[0][0]], [self.pos[0][1], self.pos[0][1]], '--r')
        self.edg_to_neighbors_plot_hadels = list()

        #self.edg_to_neighbors_plot_hadels = list()

    def update_position(self):
        next_pos = np.add(self.pos, np.multiply(self.vel, self.dt))
        if not self.env.is_in_border_polygon(next_pos):
            return
        if not self.env.is_in_obs(next_pos):
            self.pos = next_pos
        # if self.env.is_los(self.pos, next_pos):
        #     self.pos = next_pos
        # else:
        #     flag, int_point = self.env.is_in_obs(self.pos, next_pos)
        #     if flag:
        #         self.pos = int_point
        #     #else: # Note that we have a problem

    def update_velocity(self):
        des_vel = np.subtract(self.virtual_target, self.pos)
        acc = np.multiply(self.acc_factor, (des_vel - self.vel))
        acc_norm = np.linalg.norm(acc)
        if acc_norm > self.acc_lim:
            acc = np.multiply(acc / acc_norm, self.acc_lim)
        vel = self.vel + acc*self.dt
        vel_norm = np.linalg.norm(vel)
        if vel_norm > self.vel_lim:
            self.vel = np.multiply(vel / vel_norm, self.vel_lim)
        else:
            self.vel = vel

    def update_ang(self):
        self.current_heading = self.virtual_heading

    def update_state(self):
        self.update_position()
        self.update_velocity()
        self.update_ang()

    def update_virtual_targets(self, virtual_target, virtual_heading_target):
        self.virtual_target = virtual_target
        self.virtual_heading = virtual_heading_target

    def update_drone_plot(self):
        self.plot_hadel_env.set_data(self.pos[0][0], self.pos[0][1])
        self.plot_hadel_grid.set_data(self.pos[0][0], self.pos[0][1])
        self.plot_hadel_env_vt.set_data(self.virtual_target[0][0], self.virtual_target[0][1])
        self.plot_hadel_grid_vt.set_data(self.virtual_target[0][0], self.virtual_target[0][1])
        self.plot_hadel_env_l2vt.set_data([self.pos[0][0], self.virtual_target[0][0]], [self.pos[0][1], self.virtual_target[0][1]])
        self.plot_hadel_grid_l2vt.set_data([self.pos[0][0], self.virtual_target[0][0]], [self.pos[0][1], self.virtual_target[0][1]])

    def tof_sensing(self):
        output_list = list()
        directions_vec = np.add([0, np.pi/2, np.pi, 3*np.pi/2], self.current_heading)
        for phi in directions_vec:
            flag, sense_pos = self.env.TOF_sensing(self.pos, phi)
            output_list.append([flag, self.pos, sense_pos])
        return output_list

    def neighbors_sensing(self, drones_arr):
        neighbors_pos = list()
        for i in range(0, drones_arr.__len__()):
            if drones_arr[i].ID != self.ID:
                diff = np.subtract(drones_arr[i].pos, self.pos)
                if np.linalg.norm(diff) < self.neighbours_visibility_range and self.env.is_los(drones_arr[i].pos, self.pos):
                    neighbors_pos.append(diff)
        return neighbors_pos

    def preform_step(self, drone_arr):
        if not self.stop_command:
            self.update_position()
            self.update_velocity()
            self.update_ang()
        neighbors_pos = self.neighbors_sensing(drone_arr)
        self.update_drone_plot()
        return neighbors_pos

    def plot_edges(self, ax, neighbors_list):
        self.delete_edges()
        for pos in neighbors_list:
            edge = ax.plot([self.pos[0][0], self.pos[0][0]+pos[0][0]], [self.pos[0][1], self.pos[0][1]+pos[0][1]])
            self.edg_to_neighbors_plot_hadels.append(edge)

    def delete_edges(self):
        for edge_handel in self.edg_to_neighbors_plot_hadels:
            edge_handel[0].remove()
        # self.edg_to_neighbors_plot_hadels.clear() # not supported in python 2.7
        del self.edg_to_neighbors_plot_hadels[:]

