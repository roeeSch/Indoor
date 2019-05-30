import math
import numpy as np
from bresenham import bresenham

class Node:

    def __init__(self, x, y, cost, pind, xy_idx):
        self.x = x
        self.y = y
        self.cost = cost
        self.pind = pind
        self.xy_idx = xy_idx


class Astar:

    def __init__(self, scanning_range, x_lim, y_lim, matrix, res):
        self.scanning_range = scanning_range
        self.x_lim = x_lim
        self.y_lim = y_lim
        self.matrix = matrix
        self.res = res

    def PlanningAlg(self, sx, sy, gx, gy, corners_xy):

        # sx: start x position [m]
        # sy: start y position [m]
        # gx: goal x position [m]
        # gy: goal y position [m]

        mx = []
        my = []
        idx_mxy = []
        for midx, mpoint in enumerate(corners_xy):
            # mx: x position list of motion [m]
            # my: y position list of motion [m]
            if not (mpoint[0] == gx and mpoint[1] == gy):
                mx.append(mpoint[0])
                my.append(mpoint[1])
                idx_mxy.append(midx)

        nstart = Node(sx, sy, 0, -1, -1)
        ngoal = Node(gx, gy, 0, -1, -1)

        obmap, minx, miny, maxx, maxy, xw, yw = self.calc_obstacle_map()

        # fig = plt.figure(45645)
        # plt.imshow(np.transpose(obmap), origin='lower')

        g_i, g_j = self.xy_to_ij(gx, gy)
        s_i, s_j = self.xy_to_ij(sx, sy)
        if self.is_path_free(s_i, s_j, g_i, g_j, obmap):
            mx = []
            my = []
            idx_mxy = []

        openset, closedset = dict(), dict()
        openset[self.calc_index(nstart, xw, minx, miny)] = nstart

        while 1:

            try:
                c_id = min(openset, key=lambda o: openset[o].cost + self.calc_heuristic(ngoal, openset[o]))
            except:
                Astar_path = []
                path_idxs = []
                return Astar_path, path_idxs

            current = openset[c_id]

            c_i, c_j = self.xy_to_ij(current.x, current.y)
            if c_i == g_i and c_j == g_j:
                # print("Find goal")
                ngoal.pind = current.pind
                ngoal.cost = current.cost
                break

            # Remove the item from the open set
            del openset[c_id]
            # Add it to the closed set
            closedset[c_id] = current

            motion = self.get_motion_model(mx, my, current, ngoal, obmap, idx_mxy)

            # expand search grid based on motion model
            for i in range(len(motion)):
                node = Node(int(motion[i][0]),
                            int(motion[i][1]),
                            current.cost + motion[i][2], c_id, motion[i][3])

                n_id = self.calc_index(node, xw, minx, miny)

                if n_id in closedset:
                    continue

                if not self.verify_node(node, obmap, minx, miny, maxx, maxy):
                    continue

                if n_id not in openset:
                    openset[n_id] = node  # Discover a new node
                else:
                    if openset[n_id].cost >= node.cost:
                        # This path is the best until now. record it!
                        openset[n_id] = node

        path_idxs, Astar_path = self.calc_fianl_path(ngoal, closedset)

        return Astar_path, path_idxs


    def calc_heuristic(self, n1, n2):
        w = 1.0  # weight of heuristic
        d = w * math.sqrt((n1.x - n2.x) ** 2 + (n1.y - n2.y) ** 2)

        return d

    def verify_node(self, node, obmap, minx, miny, maxx, maxy):

        if node.x < minx:
            return False
        elif node.y < miny:
            return False
        elif node.x >= maxx:
            return False
        elif node.y >= maxy:
            return False

        i,j = self.xy_to_ij(node.x, node.y)
        if obmap[i][j]:
            return False

        return True

    def calc_obstacle_map(self):

        minx = self.x_lim[0]
        miny = self.y_lim[0]
        maxx = self.x_lim[1]
        maxy = self.y_lim[1]

        xwidth = round(maxx - minx)
        ywidth = round(maxy - miny)

        nrow, ncol = np.shape(self.matrix)
        # obstacle map generation
        obmap = np.zeros((nrow, ncol), dtype=bool)

        for ix in range(nrow):
            for iy in range(ncol):
                if self.matrix[ix][iy] != 1:
                    obmap[ix][iy] = True

        return obmap, minx, miny, maxx, maxy, xwidth, ywidth

    def calc_index(self, node, xwidth, xmin, ymin):
        return (node.y - ymin) * xwidth + (node.x - xmin)

    def get_motion_model(self, mx, my, nstart, ngoal, obmap, idx_mxy):

        okways = []
        allx, ally, allcost, allmidxs = [], [], [], []
        pradius = self.scanning_range
        # pradius = math.sqrt((nstart.x - ngoal.x) ** 2 + (nstart.y - ngoal.y) ** 2)
        for j, jend in enumerate(zip(mx, my)):
            dcost = math.sqrt((nstart.x - jend[0]) ** 2 + (nstart.y - jend[1]) ** 2)
            if dcost <= pradius: # Not mandatory if... Only to speed up!
                allx.append(jend[0])
                ally.append(jend[1])
                allcost.append(dcost)
                allmidxs.append(idx_mxy[j])
        allx.append(ngoal.x)
        ally.append(ngoal.y)
        allcost.append(math.sqrt((nstart.x - ngoal.x) ** 2 + (nstart.y - ngoal.y) ** 2))
        allmidxs.append(-1)
        for i, iend in enumerate(zip(allx, ally, allcost, allmidxs)):
            s_i, s_j = self.xy_to_ij(nstart.x, nstart.y)
            g_i, g_j = self.xy_to_ij(iend[0], iend[1])
            if self.is_path_free(s_i, s_j, g_i, g_j, obmap):
                okways.append([iend[0], iend[1], iend[2], iend[3]])

        return okways


    def is_path_free(self, si, sj, gi, gj, obmap):
        bpath = list(bresenham(si, sj, gi, gj))
        ok_way = True
        for ii, elem in enumerate(bpath):
            if obmap[elem[0]][elem[1]]:
                ok_way = False
                break

        return ok_way


    def calc_fianl_path(self, ngoal, closedset):
        # generate final path
        final_path = [[ngoal.x, ngoal.y]]
        path_idxs = [-1]
        # rx, ry = [ngoal.x], [ngoal.y]
        pind = ngoal.pind
        while pind != -1:
            n = closedset[pind]
            # rx.append(n.x)
            # ry.append(n.y)
            final_path.append([n.x, n.y])
            pind = n.pind
            path_idxs.append(n.xy_idx)

        return list(reversed(path_idxs)), list(reversed(final_path))

    def xy_to_ij(self, x, y):
        i = int(np.floor((x - self.x_lim[0]) / self.res))
        j = int(np.floor((y - self.y_lim[0]) / self.res))
        return i, j


def is_los(p1, p2, matrix, x_lim, y_lim, res):
    n = int(np.maximum(1, np.ceil(np.linalg.norm(p1-p2)/res)*3))
    x = np.linspace(p1[0][0], p2[0][0], num=n, endpoint=True)
    y = np.linspace(p1[0][1], p2[0][1], num=n, endpoint=True)
    for ind in range(1, n):
        i, j = xy_to_ij(x[ind], y[ind], x_lim, y_lim, res)
        if matrix[i][j] != 1:
            return False
    return True


def xy_to_ij(x, y, x_lim, y_lim, res):
    i = int(np.floor((x - x_lim[0])/res))
    j = int(np.floor((y - y_lim[0]) / res))
    return i, j


# def get_goal_point(pos, temp_interesting_points_list_xy, matrix, x_lim, y_lim, res, agents):
#     g_idx = 0
#     dist_arr = []
#     for idx, elem in enumerate(temp_interesting_points_list_xy):
#         dist_arr.append(np.linalg.norm(np.subtract(elem, pos[0])))
#     sorted_dist_idxs = sorted(range(len(dist_arr)), key=lambda k: dist_arr[k])
#     for idx in sorted_dist_idxs:
#         if is_los(pos, [[temp_interesting_points_list_xy[idx][0], temp_interesting_points_list_xy[idx][1]]], matrix, x_lim, y_lim, res):
#             g_idx = idx
#             break
#     return g_idx

def get_goal_point(pos, id, interesting_points_list_xy, matrix, x_lim, y_lim, res, agents):
    # This function chooses specific interest point for each drone

    dist_arr = []
    dist_from_prev_pos = 0
    n_tails_between_drones = 15

    for ielem, elem in enumerate(interesting_points_list_xy):
        dist_arr.append(np.linalg.norm(np.subtract(elem, pos[0])))
    sorted_dist_idxs = sorted(range(len(dist_arr)), key=lambda k: dist_arr[k])

    valid_wp_flag = False
    g_idx = np.random.randint(len(interesting_points_list_xy))
    if len(agents) > 1: # For multiple drones
        i_s, j_s = xy_to_ij(pos[0][0], pos[0][1], x_lim, y_lim, res)
        for idx in sorted_dist_idxs:
            i_g, j_g = xy_to_ij(interesting_points_list_xy[idx][0], interesting_points_list_xy[idx][1], x_lim, y_lim, res)
            if np.linalg.norm(np.subtract([i_s, j_s],[i_g, j_g])) > dist_from_prev_pos:
                for i_agent in range(len(agents)):
                    if agents[i_agent].ID != id:
                        add_drone_pos = agents[i_agent].current_pos
                        i_p_s, j_p_s = xy_to_ij(add_drone_pos[0][0], add_drone_pos[0][1], x_lim, y_lim, res)
                        if np.linalg.norm(np.subtract([i_g, j_g], [i_p_s, j_p_s])) > n_tails_between_drones:
                            if is_los(pos,[[interesting_points_list_xy[idx][0], interesting_points_list_xy[idx][1]]],
                                      matrix, x_lim, y_lim, res):
                                g_idx = idx
                                valid_wp_flag = True
                                break
                if valid_wp_flag == True:
                    break

        if valid_wp_flag == False:

            for idx in sorted_dist_idxs:
                i_g, j_g = xy_to_ij(interesting_points_list_xy[idx][0], interesting_points_list_xy[idx][1], x_lim, y_lim,
                                    res)
                if np.linalg.norm(np.subtract([i_s, j_s], [i_g, j_g])) > dist_from_prev_pos:
                    for i_agent in range(len(agents)):
                        if agents[i_agent].ID != id:
                            add_drone_pos = agents[i_agent].current_pos
                            i_p_s, j_p_s = xy_to_ij(add_drone_pos[0][0], add_drone_pos[0][1], x_lim, y_lim, res)
                            add_drone_next_pos = agents[i_agent].next_pos
                            i_p_g, j_p_g = xy_to_ij(add_drone_next_pos[0][0], add_drone_next_pos[0][1], x_lim, y_lim, res)
                            if np.linalg.norm(np.subtract([i_g, j_g], [i_p_s, j_p_s])) > n_tails_between_drones and \
                                    np.linalg.norm(np.subtract([i_g, j_g], [i_p_g, j_p_g])) > n_tails_between_drones:
                                g_idx = idx
                                valid_wp_flag = True
                                break
                    if valid_wp_flag == True:
                        break
    else: # For one drone
        g_idx = 0
        for idx in sorted_dist_idxs:
            if is_los(pos,[[interesting_points_list_xy[idx][0], interesting_points_list_xy[idx][1]]],
                    matrix, x_lim, y_lim, res):
                g_idx = idx
                break
    return g_idx


def build_trj(pos, id, scanning_range, x_lim, y_lim, res, matrix, temp_corner_points_list_xy, temp_interesting_points_list_xy, ref_pos, agents):


    Astar_Movement = []
    astar = Astar(scanning_range, x_lim, y_lim, matrix, res)

    if len(temp_interesting_points_list_xy) > 0:

        g_idx = get_goal_point(pos, id, temp_interesting_points_list_xy, matrix, x_lim, y_lim, res, agents)
        gx = temp_interesting_points_list_xy[g_idx][0]
        gy = temp_interesting_points_list_xy[g_idx][1]

        del temp_interesting_points_list_xy[g_idx]

        astar_movement, corner_idxs = astar.PlanningAlg(pos[0][0], pos[0][1], gx, gy, temp_corner_points_list_xy)

        if len(corner_idxs) >= 3:
            for ci in corner_idxs:
                if ci != -1:
                    temp_corner_points_list_xy[ci] = []

            temp_corner_points_list_xy = filter(None, temp_corner_points_list_xy)

        Astar_Movement.append(astar_movement[1:])
    else:
        Astar_Movement.append([])

    return Astar_Movement, temp_corner_points_list_xy, temp_interesting_points_list_xy