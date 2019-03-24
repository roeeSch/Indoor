
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

    def change_tail_to_close_area(self, i, j):
        self.change_tail_color_ij(i, j, 'y')
        self.matrix[i][j] = 3

    def is_los(self, p1, p2):
        n = int(np.maximum(1, np.ceil(np.linalg.norm(p1-p2)/self.res)*3))
        x = np.linspace(p1[0][0], p2[0][0], num=n, endpoint=True)
        y = np.linspace(p1[0][1], p2[0][1], num=n, endpoint=True)
        for ind in range(1, n):
            i, j = self.xy_to_ij(x[ind], y[ind])
            if self.matrix[i][j] != 1 and self.matrix[i][j] != 3:
                return False
        return True

    def update_with_radial_sensor(self, p, r, env): # This Function is not relevant for the real system
        base_unit_vector = [[0, 1]]
        for theta in np.linspace(0, 2*np.pi, num=10, endpoint=False):
            unit_vector = [[base_unit_vector[0][0]*np.cos(theta) - base_unit_vector[0][1]*np.sin(theta),
                            base_unit_vector[0][0] * np.sin(theta) + base_unit_vector[0][1] * np.cos(theta)]]
            for dr in np.linspace(0, r, num=r / self.res * 2, endpoint=True):
                p2 = p + [[unit_vector[0][0]*dr, unit_vector[0][1]*dr]]
                i, j = self.xy_to_ij(p2[0][0], p2[0][1])
                if 0 <= i and i < self.matrix.shape[0] and 0 <= j and j < self.matrix.shape[1]:
                    if self.matrix[i][j] == 2:
                        break
                    if env.is_in_obs(p2):
                        self.change_tail_to_wall(i, j)
                        break
                    elif not(self.is_in_close_area(i, j)):
                        self.change_tail_to_empty(i, j)

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

    def non_scaned_list(self, p, r, env):
        non_sc_list = list()
        base_unit_vector = [[0, 1]]
        for theta in np.linspace(0, 2*np.pi, num=10, endpoint=False):
            unit_vector = [[base_unit_vector[0][0]*np.cos(theta) - base_unit_vector[0][1]*np.sin(theta),
                            base_unit_vector[0][0] * np.sin(theta) + base_unit_vector[0][1] * np.cos(theta)]]
            for dr in np.linspace(self.res, r, num=r / self.res * 2, endpoint=True):
                p2 = p + [[unit_vector[0][0]*dr, unit_vector[0][1]*dr]]
                i, j = self.xy_to_ij(p2[0][0], p2[0][1])
                if 0 <= i and i < self.matrix.shape[0] and 0 <= j and j < self.matrix.shape[1]:
                    if self.matrix[i][j] == 2:
                        break
                    else:
                        if self.matrix[i][j] == 0:
                            non_sc_list.append([self.ij_to_xy(i, j)])
        return non_sc_list

    def is_tail_interesting(self, i, j):
        # Tail is explored
        if self.matrix[i][j] != 1:
            return False
        # Tail not on the border of the grid
        if i < 1 or i >= self.matrix.shape[0] or j < 1 or j >= self.matrix.shape[1]:
            return False
        # Tail on the edge of explored area
        ind_list = [[-1, -1], [0, -1], [1, -1], [1, 0], [1, 1], [0, 1], [1, -1], [0, -1], [-1, -1], [0, -1]]
        for k in range(1, ind_list.__len__()-1): #TODO: need refinement
            if self.matrix[ind_list[k][0] + i][ind_list[k][1] + j] == 0:
                return True
        return False

        # Tail in a corner

    def is_tail_at_outer_corner(self, i, j):
        if self.matrix[i][j] == 1:
            ind_list = [[[-1, 0], [-1, -1], [0, -1]],
                        [[0, -1], [1, -1], [1, 0]],
                        [[1, 0], [1, 1], [0, 1]],
                        [[0, 1], [-1, 1], [-1, 0]]]
            for k in range(0, ind_list.__len__()):
                if (self.matrix[ind_list[k][0][0] + i][ind_list[k][0][1] + j] == 1 and
                        (self.matrix[ind_list[k][1][0] + i][ind_list[k][1][1] + j] >= 2 or self.matrix[ind_list[k][1][0] + i][ind_list[k][1][1] + j] == 0) and
                        self.matrix[ind_list[k][2][0] + i][ind_list[k][2][1] + j] == 1):
                    return True
        return False

    def find_outer_corners_tails(self):
        tail_list = list()
        for i in range(1, self.matrix.__len__() - 1):
            for j in range(1, self.matrix[i].__len__() - 1):
                if self.is_tail_at_outer_corner(i, j):
                    tail_list.append([i, j])
        self.outer_corner_tail_list = tail_list

    def is_tail_at_inner_corner(self, i, j):
        if self.matrix[i][j] == 1:
            ind_list = [[[-1, 0], [-1, -1], [0, -1]],
            [[0, -1], [1, -1], [1, 0]],
            [[1, 0], [1, 1], [0, 1]],
            [[0, 1], [-1, 1], [-1, 0]]]
            for k in range(0, ind_list.__len__()):
                if (self.matrix[ind_list[k][0][0] + i][ind_list[k][0][1] + j] >= 2 and
                #self.matrix[ind_list[k][1][0] + i][ind_list[k][1][1] + j] >= 2 and
                self.matrix[ind_list[k][2][0] + i][ind_list[k][2][1] + j] >= 2):
                    return True
        return False

    def find_inner_corners_tails(self):
        tail_list = list()
        for i in range(1, self.matrix.__len__() - 1):
            for j in range(1, self.matrix[i].__len__() - 1):
                if self.is_tail_at_inner_corner(i, j):
                    tail_list.append([i, j])
        return tail_list

    def update_close_areas(self):
        flag = False
        inner_corners_tails_list = self.find_inner_corners_tails()
        for i1, j1 in inner_corners_tails_list:

            for i2, j2 in inner_corners_tails_list:
                flag = False
                # checking for wall on the right or on the left
                if i1 == i2 and j1 != j2:

                    if j1 < j2:
                        j_ind = range(j1, j2 + 1)
                    else:
                        j_ind = range(j2, j1 + 1)
                    #checking for wall on the right
                    if (self.matrix[i1+1][j1] == 2 or self.matrix[i1+1][j1] == 3) and (self.matrix[i1+1][j2] == 2 or self.matrix[i1+1][j2] == 3):
                        flag = True
                        for j in j_ind:
                            if not(self.matrix[i1+1][j] == 2 or self.matrix[i1+1][j] == 3) or self.matrix[i1][j] != 1:
                                flag = False
                                break

                    # checking for wall on the left
                    if (self.matrix[i1 - 1][j1] == 2 or self.matrix[i1 - 1][j1] == 3) and (
                            self.matrix[i1 - 1][j2] == 2 or self.matrix[i1 - 1][j2] == 3):
                        flag = True
                        for j in j_ind:
                            if not (self.matrix[i1 - 1][j] == 2 or self.matrix[i1 - 1][j] == 3) or self.matrix[i1][j] != 1:
                                flag = False
                                break
                    if flag:
                        for j in j_ind:
                            if self.matrix[i1,j] == 2:
                                a = 1
                            self.change_tail_to_close_area(i1, j)

                # checking for wall on the up or on the bottom
                if i1 != i2 and j1 == j2:

                    if i1 < i2:
                        i_ind = range(i1, i2 + 1)
                    else:
                        i_ind = range(i2, i1 + 1)
                    # checking for wall on the upper side
                    if (self.matrix[i1][j1+1] == 2 or self.matrix[i1][j1+1] == 3) and (
                            self.matrix[i2][j1+1] == 2 or self.matrix[i2][j1+1] == 3):
                        flag = True
                        for i in i_ind:
                            if not (self.matrix[i][j1+1] == 2 or self.matrix[i][j1+1] == 3) or self.matrix[i][j1] != 1:
                                flag = False
                                break

                    # checking for wall on the bottom side
                    elif (self.matrix[i1][j1-1] == 2 or self.matrix[i1][j1-1] == 3) and (
                            self.matrix[i2][j1-1] == 2 or self.matrix[i2][j1-1] == 3):
                        flag = True
                        for i in i_ind:
                            if not (self.matrix[i][j1-1] == 2 or self.matrix[i][j1-1] == 3) or self.matrix[i][j1] != 1:
                                flag = False
                                break
                    if flag:
                        for i in i_ind:
                            if self.matrix[i,j1] == 2:
                                a = 1
                            self.change_tail_to_close_area(i, j1)

    def is_in_close_area(self, i, j):
        return self.matrix[i][j] == 3

    def is_in_close_area_xy(self, p):
        i, j = self.xy_to_ij(p[0][0], p[0][1])
        return self.matrix[i][j] == 3

    def find_interesting_tail(self):
        tail_list = list()
        for i in range(1, self.matrix.__len__()-1):
            for j in range(1, self.matrix[i].__len__()-1):
                if self.is_tail_interesting(i, j):
                    tail_list.append([i, j])
        return tail_list

    def mark_enterence_as_closed_area(self, env):
        for i in range(0, self.matrix.__len__()):
            x, y = self.ij_to_xy(i, 0)
            if not(env.is_in_obs([[x, y]])):
                self.change_tail_to_close_area(i, 0)
            x, y = self.ij_to_xy(i, self.matrix[0].__len__()-1)
            if not (env.is_in_obs([[x, y]])):
                self.change_tail_to_close_area(i, self.matrix[0].__len__()-1)
        for j in range(0, self.matrix[0].__len__()):
            x, y = self.ij_to_xy(0, j)
            if not(env.is_in_obs([[x, y]])):
                self.change_tail_to_close_area(0, j)
            x, y = self.ij_to_xy(self.matrix.__len__()-1, j)
            if not (env.is_in_obs([[x, y]])):
                self.change_tail_to_close_area(self.matrix.__len__()-1, j)

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

    def is_step_legal(self, curr_pos, step):
        new_pos = curr_pos + step
        i, j = self.xy_to_ij(new_pos[0][0], new_pos[0][1])
        if not (0 <= i and i < self.matrix.shape[0] and 0 <= j and j < self.matrix.shape[1] and (self.matrix[i][j] == 1 or self.matrix[i][j] == 3)):
            return False
        if (not self.is_in_close_area_xy(curr_pos)) and self.is_in_close_area_xy(new_pos): #TODO: Consider to remove
            return False
        return self.is_los(curr_pos, new_pos)


    def complete_wall_in_corners(self):
        for i in range(1, self.matrix.__len__()-1):
            for j in range(1, self.matrix[i].__len__()-1):
                if self.matrix[i][j] == 0:
                    if ((self.matrix[i - 1][j] == 2 and self.matrix[i][j - 1] == 2 and (self.matrix[i - 1][j - 1] == 1 or self.matrix[i - 1][j - 1] == 3)) or
                        (self.matrix[i + 1][j] == 2 and self.matrix[i][j - 1] == 2 and (self.matrix[i + 1][j - 1] == 1 or self.matrix[i - 1][j - 1] == 3)) or
                        (self.matrix[i + 1][j] == 2 and self.matrix[i][j + 1] == 2 and (self.matrix[i + 1][j + 1] == 1 or self.matrix[i - 1][j + 1] == 3)) or
                        (self.matrix[i - 1][j] == 2 and self.matrix[i][j + 1] == 2 and (self.matrix[i - 1][j + 1] == 1 or self.matrix[i - 1][j + 1] == 3))):
                        self.change_tail_to_wall(i, j)
        j = 0
        for i in range(1, self.matrix.__len__()-1):
            if (self.matrix[i][j] == 0 and
                    (self.matrix[i + 1][j] == 2 and self.matrix[i][j + 1] == 2 and (self.matrix[i + 1][j + 1] == 1 or self.matrix[i + 1][j + 1] == 3)) or
                    (self.matrix[i - 1][j] == 2 and self.matrix[i][j + 1] == 2 and (self.matrix[i - 1][j + 1] == 1 or self.matrix[i - 1][j + 1] == 3))):
                self.change_tail_to_wall(i, j)
        j = self.matrix[0].__len__()-1
        for i in range(1, self.matrix.__len__() - 1):
            if (self.matrix[i][j] == 0 and
                    (self.matrix[i - 1][j] == 2 and self.matrix[i][j - 1] == 2 and (self.matrix[i - 1][j - 1] == 1 or self.matrix[i - 1][j - 1] == 3)) or
                    (self.matrix[i + 1][j] == 2 and self.matrix[i][j - 1] == 2 and (self.matrix[i + 1][j - 1] == 1 or self.matrix[i + 1][j - 1] == 3))):
                self.change_tail_to_wall(i, j)
        i = 0
        for j in range(1, self.matrix[0].__len__()-1):
            if (self.matrix[i][j] == 0 and
                    (self.matrix[i + 1][j] == 2 and self.matrix[i][j - 1] == 2 and (self.matrix[i + 1][j - 1] == 1 or self.matrix[i - 1][j - 1] == 3)) or
                    (self.matrix[i + 1][j] == 2 and self.matrix[i][j + 1] == 2 and (self.matrix[i + 1][j + 1] == 1 or self.matrix[i + 1][j - 1] == 3))):
                self.change_tail_to_wall(i, j)
        i = self.matrix.__len__()-1
        for j in range(1, self.matrix[0].__len__() - 1):
            if (self.matrix[i][j] == 0 and
                    (self.matrix[i - 1][j] == 2 and self.matrix[i][j - 1] == 2 and (self.matrix[i - 1][j - 1] == 1 or self.matrix[i - 1][j - 1] == 3)) or
                    (self.matrix[i - 1][j] == 2 and self.matrix[i][j + 1] == 2 and (self.matrix[i - 1][j + 1] == 1 or self.matrix[i - 1][j + 1] == 3))):
                self.change_tail_to_wall(i, j)
        if (self.matrix[0][0] == 0 and
                    (self.matrix[0][1] == 2 and self.matrix[1][0] == 2)):
                self.change_tail_to_wall(0, 0)
        if (self.matrix[0][self.matrix[0].__len__()-1] == 0 and
                (self.matrix[0][self.matrix[0].__len__()-2] == 2 and self.matrix[1][self.matrix[0].__len__()-1] == 2)):
            self.change_tail_to_wall(0, self.matrix[0].__len__()-1)
        if (self.matrix[self.matrix.__len__()-1][self.matrix[0].__len__()-1] == 0 and
                (self.matrix[self.matrix[0].__len__()-1][self.matrix[0].__len__()-2] == 2 and self.matrix[self.matrix[0].__len__()-2][self.matrix[0].__len__()-1] == 2)):
            self.change_tail_to_wall(self.matrix.__len__()-1, self.matrix[0].__len__()-1)
        if (self.matrix[self.matrix.__len__()-1][0] == 0 and
                (self.matrix[self.matrix[0].__len__()-1][1] == 2 and self.matrix[self.matrix[0].__len__()-2][0] == 2)):
            self.change_tail_to_wall(self.matrix.__len__()-1, 0)
