import numpy as np

class Grid:

    def __init__(self, border_polygon, res):
        self.x_lim = [border_polygon[0][0], border_polygon[0][0]]
        self.y_lim = [border_polygon[0][1], border_polygon[0][1]]
        for i in range(1, border_polygon.__len__()):
            if self.x_lim[0] > border_polygon[i][0]:
                self.x_lim[0] = border_polygon[i][0]
            if self.x_lim[1] < border_polygon[i][0]:
                self.x_lim[1] = border_polygon[i][0]
            if self.y_lim[0] > border_polygon[i][1]:
                self.y_lim[0] = border_polygon[i][1]
            if self.y_lim[1] < border_polygon[i][1]:
                self.y_lim[1] = border_polygon[i][1]

        self.res = res
        self.matrix = np.zeros([np.int64(np.ceil((self.x_lim[1]-self.x_lim[0])/self.res)), np.int64(np.ceil((self.y_lim[1]-self.y_lim[0])/self.res))])
        self.empty_idxs = []
        self.wall_idxs = []


    def xy_to_ij(self, x, y):
        i = int(np.floor((x - self.x_lim[0])/self.res))
        j = int(np.floor((y - self.y_lim[0]) / self.res))
        return i, j

    def ij_to_xy(self, i, j):
        x = self.x_lim[0] + i*self.res + self.res/2
        y = self.y_lim[0] + j*self.res + self.res/2
        return x, y

    def change_tail_to_empty(self, i, j):
        # self.change_tail_color_ij(i, j, 'k')
        self.matrix[i][j] = 1

    def change_tail_to_wall(self, i, j):
        # self.change_tail_color_ij(i, j, 'w')
        self.matrix[i][j] = 2

    def update_from_tof_sensing_list(self, tof_sensing_list):
        self.empty_idxs = []
        self.wall_idxs = []
        for ln in tof_sensing_list:
            self.update_with_tof_sensor(ln[1], ln[2], ln[0])

    def update_with_tof_sensor(self, sensor_pos, tof_sensing_pos, is_tof_senses):
        num_of_samples = int(np.floor(np.linalg.norm(tof_sensing_pos - sensor_pos) / self.res * 2))
        xs = np.linspace(sensor_pos[0][0], tof_sensing_pos[0][0], num=num_of_samples, endpoint=True)
        ys = np.linspace(sensor_pos[0][1], tof_sensing_pos[0][1], num=num_of_samples, endpoint=True)
        for ind in range(1, num_of_samples):
            i, j = self.xy_to_ij(xs[ind], ys[ind])
            if 0 > i or i >= self.matrix.shape[0] or 0 > j or j >= self.matrix.shape[1]:
                return
            if self.matrix[i][j] == 0:
                self.change_tail_to_empty(i, j)
                self.empty_idxs.append([i,j])
        if is_tof_senses:
            d = tof_sensing_pos - sensor_pos
            norm_d = np.linalg.norm(d)
            if norm_d > 0:
                wall_pos = tof_sensing_pos + d /norm_d*self.res/1000
                i, j = self.xy_to_ij(wall_pos[0][0], wall_pos[0][1])
                self.change_tail_to_wall(i, j)
                self.wall_idxs.append([i, j])

