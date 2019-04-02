
import numpy as np
from shapely.geometry import Point, LineString, Polygon
from descartes import PolygonPatch

class Grid:

    def __init__(self, border_polygon, res, ax):
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
        self.ax = ax
        self.tail_handles = list()
        self.outer_corner_tail_list = list()


    def xy_to_ij(self, x, y):
        i = int(np.floor((x - self.x_lim[0])/self.res))
        j = int(np.floor((y - self.y_lim[0]) / self.res))
        return i, j

    def ij_to_xy(self, i, j):
        x = self.x_lim[0] + i*self.res + self.res/2
        y = self.y_lim[0] + j*self.res + self.res/2
        return x, y

    def plot_ij(self, i, j):
        pol_center = self.ij_to_xy(i, j)
        tail = Polygon([(pol_center[0]-self.res/2, pol_center[1]-self.res/2), (pol_center[0]-self.res/2, pol_center[1]+self.res/2)
                       , (pol_center[0]+self.res/2, pol_center[1]+self.res/2), (pol_center[0]+self.res/2, pol_center[1]-self.res/2)
                       , (pol_center[0]-self.res/2, pol_center[1]-self.res/2)])
        return self.ax.add_patch(PolygonPatch(tail, facecolor='gray'))

    def plot_grid(self):
        for i in range(0, self.matrix.__len__()):
            handles_list = list()
            for j in range(0, self.matrix[i].__len__()):
                handles_list.append(self.plot_ij(i, j))
            self.tail_handles.append(handles_list)

    def change_tail_color_ij(self, i, j, color):
        self.tail_handles[i][j].set_fc(color)

    def change_tail_to_empty(self, i, j):
        self.change_tail_color_ij(i, j, 'k')
        self.matrix[i][j] = 1

    def change_tail_to_wall(self, i, j):
        self.change_tail_color_ij(i, j, 'w')
        self.matrix[i][j] = 2

    def update_from_tof_sensing_list(self, tof_sensing_list):
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
        if is_tof_senses:
            d = tof_sensing_pos - sensor_pos
            norm_d = np.linalg.norm(d)
            if norm_d > 0:
                wall_pos = tof_sensing_pos + d /norm_d*self.res/1000
                i, j = self.xy_to_ij(wall_pos[0][0], wall_pos[0][1])
                self.change_tail_to_wall(i, j)

    def change_tail_list_color(self, tail_list, color):
        for i, j in tail_list:
            self.change_tail_color_ij(i, j, color)

    def plot_points_on_tails_in_list(self, tail_list, color):
        handel_list = list()
        for i, j in tail_list:
            x, y = self.ij_to_xy(i, j)
            h = self.ax.plot(x, y, color)
            handel_list.append(h)
        return handel_list

    def plot_interesting_points(self, interesting_points_list_ij):
        for i, j in interesting_points_list_ij:
            self.change_tail_color_ij( i, j, 'm')

    def erase_interesting_points(self, interesting_points_list_ij):
        for i, j in interesting_points_list_ij:
            self.change_tail_to_empty(i, j)

    def erase_corner_points(self, corner_points_list_ij):
        for i, j in corner_points_list_ij:
            self.change_tail_to_empty(i, j)

    def plot_corner_points(self, corner_points_list_ij):
        for i, j in corner_points_list_ij:
            self.change_tail_color_ij(i, j, 'g')

    def complete_wall_corner(self, wall_idxs_ij):
        for wall_idx in wall_idxs_ij:
            self.change_tail_to_wall(wall_idx[0], wall_idx[1])


