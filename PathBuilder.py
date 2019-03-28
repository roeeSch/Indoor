import math
import numpy as np
from bresenham import bresenham

class Node:

    def __init__(self, x, y, cost, pind):
        self.x = x
        self.y = y
        self.cost = cost
        self.pind = pind


class Astar:

    def __init__(self, grid_motion, scanning_range, grid):
        self.grid_motion = grid_motion
        self.scanning_range = scanning_range
        self.grid = grid

    def PlanningAlg(self, sx, sy, gx, gy):

        # sx: start x position [m]
        # sy: start y position [m]
        # gx: goal x position [m]
        # gy: goal y position [m]

        mx = []
        my = []
        for mpoint in self.grid.corner_points_list_xy:
            # mx: x position list of motion [m]
            # my: y position list of motion [m]
            if mpoint[0] != gx and mpoint[1] != gy:
                mx.append(mpoint[0])
                my.append(mpoint[1])

        nstart = Node(sx, sy, 0, -1)
        ngoal = Node(gx, gy, 0, -1)

        obmap, minx, miny, maxx, maxy, xw, yw = self.calc_obstacle_map()

        # fig = plt.figure(45645)
        # plt.imshow(np.transpose(obmap), origin='lower')

        g_i, g_j = self.grid.xy_to_ij(gx, gy)
        s_i, s_j = self.grid.xy_to_ij(sx, sy)
        if self.is_path_free(s_i, s_j, g_i, g_j, obmap):
            mx = []
            my = []

        openset, closedset = dict(), dict()
        openset[self.calc_index(nstart, xw, minx, miny)] = nstart

        while 1:

            try:
                c_id = min(openset, key=lambda o: openset[o].cost + self.calc_heuristic(ngoal, openset[o]))
            except:
                Astar_path = []
                return Astar_path

            current = openset[c_id]

            c_i, c_j = self.grid.xy_to_ij(current.x, current.y)
            if c_i == g_i and c_j == g_j:
                # print("Find goal")
                ngoal.pind = current.pind
                ngoal.cost = current.cost
                break

            # Remove the item from the open set
            del openset[c_id]
            # Add it to the closed set
            closedset[c_id] = current

            motion = self.get_motion_model(mx, my, current, ngoal, self.grid_motion, obmap)

            # expand search grid based on motion model
            for i in range(len(motion)):
                if self.grid_motion:
                    node = Node(current.x + motion[i][0],
                                current.y + motion[i][1],
                                current.cost + motion[i][2], c_id)
                else:
                    node = Node(int(motion[i][0]),
                                int(motion[i][1]),
                                current.cost + motion[i][2], c_id)

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

        Astar_path = self.calc_fianl_path(ngoal, closedset)

        return Astar_path

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

        i,j = self.grid.xy_to_ij(node.x, node.y)
        if obmap[i][j]:
            return False

        return True

    def calc_obstacle_map(self):

        minx = self.grid.x_lim[0]
        miny = self.grid.y_lim[0]
        maxx = self.grid.x_lim[1]
        maxy = self.grid.y_lim[1]

        xwidth = round(maxx - minx)
        ywidth = round(maxy - miny)

        nrow, ncol = np.shape(self.grid.matrix)
        # obstacle map generation
        obmap = np.zeros((nrow, ncol), dtype=bool)

        for ix in range(nrow):
            for iy in range(ncol):
                if self.grid.matrix[ix][iy] != 1:
                    obmap[ix][iy] = True

        return obmap, minx, miny, maxx, maxy, xwidth, ywidth

    def calc_index(self, node, xwidth, xmin, ymin):
        return (node.y - ymin) * xwidth + (node.x - xmin)

    def get_motion_model(self, mx, my, nstart, ngoal, grid_motion, obmap):

        okways = []
        if grid_motion:

            okways = [[1, 0, 1],
                      [0, 1, 1],
                      [-1, 0, 1],
                      [0, -1, 1],
                      [-1, -1, math.sqrt(2)],
                      [-1, 1, math.sqrt(2)],
                      [1, -1, math.sqrt(2)],
                      [1, 1, math.sqrt(2)]]

        else:

            allx, ally, allcost = [], [], []
            pradius = self.scanning_range
            # pradius = math.sqrt((nstart.x - ngoal.x) ** 2 + (nstart.y - ngoal.y) ** 2)
            for j, jend in enumerate(zip(mx, my)):
                dcost = math.sqrt((nstart.x - jend[0]) ** 2 + (nstart.y - jend[1]) ** 2)
                if dcost <= pradius: # Not mandatory if... Only to speed up!
                    allx.append(jend[0])
                    ally.append(jend[1])
                    allcost.append(dcost)
            allx.append(ngoal.x)
            ally.append(ngoal.y)
            allcost.append(math.sqrt((nstart.x - ngoal.x) ** 2 + (nstart.y - ngoal.y) ** 2))
            for i, iend in enumerate(zip(allx, ally, allcost)):
                s_i, s_j = self.grid.xy_to_ij(nstart.x, nstart.y)
                g_i, g_j = self.grid.xy_to_ij(iend[0], iend[1])
                if self.is_path_free(s_i, s_j, g_i, g_j, obmap):
                    okways.append([iend[0], iend[1], iend[2]])

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
        # rx, ry = [ngoal.x], [ngoal.y]
        pind = ngoal.pind
        while pind != -1:
            n = closedset[pind]
            # rx.append(n.x)
            # ry.append(n.y)
            final_path.append([n.x, n.y])
            pind = n.pind

        return list(reversed(final_path))


def build_trj(grid, drones):

    Astar_Movement = []
    grid.erase_corner_points()
    grid.erase_interesting_points()
    grid.find_corner_points()
    grid.find_interesting_points()
    grid.plot_corner_points()
    grid.plot_interesting_points()

    for idx in range(len(drones)):

        if len(grid.interesting_points_list_xy) > 0:
            if np.random.rand(1) < 0.1:
                g_idx = np.random.randint(len(grid.interesting_points_list_xy))
            else:
                g_idx = 0

            gx = grid.interesting_points_list_xy[g_idx][0]
            gy = grid.interesting_points_list_xy[g_idx][1]

            if (np.linalg.norm(drones[idx].pos[0] - [gx, gy]) < (1.5 * drones[idx].step_noise_size)) and len(grid.interesting_points_list_xy) >= 1:
                g_idx = np.random.randint(len(grid.interesting_points_list_xy))
                gx = grid.interesting_points_list_xy[g_idx][0]
                gy = grid.interesting_points_list_xy[g_idx][1]

            grid.change_tail_to_empty(grid.interesting_points_list_ij[g_idx][0], grid.interesting_points_list_ij[g_idx][1])
            del grid.interesting_points_list_xy[g_idx]
            del grid.interesting_points_list_ij[g_idx]

            astar = Astar(0, drones[idx].scanning_range, grid)
            astar_movement = astar.PlanningAlg(drones[idx].pos[0][0], drones[idx].pos[0][1], gx, gy)

            Astar_Movement.append(astar_movement[1:])
        else:
            Astar_Movement.append([])
            continue

    return grid, Astar_Movement