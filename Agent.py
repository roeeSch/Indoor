import numpy as np

class Agent:

    def __init__(self, AgentID, pos, res, x_lim, y_lim):
        self.ID = AgentID
        self.agent_alive = True
        self.is_homing = False
        self.velocityFactor = 50
        self.step_noise_size = 20
        self.step_snr = 1
        self.stepSizeLimit = 30
        self.step_factor = 1
        self.next_pos = pos
        self.current_pos = self.next_pos
        self.next_heading = 0
        self.current_heading = self.next_heading
        self.VisibilityRange = 300
        self.scanning_range = 200
        self.repulse_range = self.VisibilityRange/10
        self.pull_range = self.VisibilityRange*4/5
        self.goal_orianted_flag_flip_prob = 0
        self.goal_orianted_flag = True #np.random.rand(1) < self.goal_orianted_flag_flip_prob
        self.reduced_neigbours_pos_list = list()
        self.astar_path = []
        self.x_lim = x_lim
        self.y_lim = y_lim
        self.res = res
        self.dist_factor = 1


    def update_current_state(self, current_pos, current_heading):
        self.current_pos = current_pos
        self.current_heading = current_heading


    def get_virtual_target_and_heading(self):
        return self.next_pos, self.next_heading


    def preform_step_sys_sim(self, current_pos, current_heading, matrix, next_heading):
        self.update_current_state(current_pos, current_heading)
        # self.reduced_neigbours_pos_list = self.neighborhood_reduction(neigbours_pos_list, matrix)
        # self.Dynam_Search_in_maze(self.reduced_neigbours_pos_list, matrix)
        self.Dynam_Search_in_maze(matrix)
        self.next_heading = next_heading


    def Dynam_Search_in_maze(self, matrix):

        max_count_val = 15
        break_counter = 0
        vec = np.zeros(2)
        flag = False
        tails_from_wall = 1
        as_flag = False
        close_wall = False

        # If there are steps left in the path and the next step is in line of sight then choose it.
        if self.astar_path != [] and self.is_step_legal(self.current_pos,
                                                        np.subtract(self.astar_path[0], self.current_pos[0]), matrix):
            vec = np.subtract(self.astar_path[0], self.current_pos[0])
            as_flag = True
        # If there are steps left in the path and the previous step is not finished and still legal then resume the prevoius step
        elif self.astar_path != [] and np.linalg.norm(
                np.subtract(self.current_pos[0], self.next_pos[0])) > self.dist_factor * self.step_noise_size \
                and self.is_step_legal(self.current_pos, np.subtract(self.next_pos[0], self.current_pos[0]), matrix):
            vec = np.subtract(self.next_pos[0], self.current_pos[0])
        # If there are no steps in path and the previous step is still legal then resume the previous step
        elif self.is_step_legal(self.current_pos, np.subtract(self.next_pos[0], self.current_pos[0]), matrix):
            vec = np.subtract(self.next_pos[0], self.current_pos[0])

        # Check if the choosen step will be to close to a wall
        if sum(vec) != 0:
            ivec, jvec = self.xy_to_ij(vec[0], vec[1])
            for ti in range(ivec - tails_from_wall, ivec + tails_from_wall + 1):
                for tj in range(jvec - tails_from_wall, jvec - tails_from_wall + 1):
                    if matrix[ti][tj] == 1:
                        close_wall = True
                        break

        # If indeed it is to close to a wall then move in the same direction but stop a few tail before the wall
        if close_wall:
            if np.linalg.norm(np.subtract(self.current_pos[0], self.next_pos[0])) > self.res:
                step = np.multiply(np.divide(vec, np.linalg.norm(vec)),
                                   np.linalg.norm(vec) - (tails_from_wall * self.res))
                if (np.linalg.norm(vec) - (tails_from_wall * self.res)) > 0:
                    vec = step

        # Limit the step size to maximum distance
        if np.linalg.norm(vec) > self.stepSizeLimit:
            temp = np.divide(vec, np.linalg.norm(vec))
            vec = np.multiply(temp, self.stepSizeLimit)

        while break_counter < max_count_val:
            break_counter = break_counter + 1
            step = self.step_noise_size * ([0.5, 0.5] - np.random.rand(2)) + vec
            if self.is_step_legal(self.current_pos, step, matrix):
                break

        if break_counter < max_count_val:
            self.next_pos = self.current_pos + step
            if as_flag and self.astar_path != []:
                del self.astar_path[0]


# This is the important function, that should be rewriten
    def Dynam_Search_in_maze_old(self, NeighborsPosList, matrix):
        flag = False
        break_counter = 0
        rep_att_vec = np.zeros(2)

        # Goal oriented movement
        noise_fac = 1
        if self.goal_orianted_flag:
            optinal_goal_list = self.non_scaned_list(self.current_pos, 5 * self.scanning_range, matrix)
            if optinal_goal_list.__len__() > 0:
                vec = optinal_goal_list[0] - self.current_pos
                goal_vec = vec / np.linalg.norm(vec)
                rep_att_vec = rep_att_vec + goal_vec
            else:
                noise_fac = 5
        else:
            noise_fac = 2
        # Neighbours oriented movement
        for NeighborPos in NeighborsPosList:
            #rep_att_vec = rep_att_vec - NeighborPos / np.linalg.norm(NeighborPos)
            if np.linalg.norm(NeighborPos) < self.repulse_range:
                rep_att_vec = rep_att_vec - NeighborPos / np.linalg.norm(NeighborPos) * self.VisibilityRange
            if np.linalg.norm(NeighborPos) > self.pull_range*noise_fac:
                rep_att_vec = rep_att_vec + NeighborPos / np.linalg.norm(NeighborPos) * self.VisibilityRange
        if np.linalg.norm(rep_att_vec) > 0:
            rep_att_vec = rep_att_vec/np.linalg.norm(rep_att_vec)
        if np.random.rand() < self.goal_orianted_flag_flip_prob:
            self.goal_orianted_flag = not self.goal_orianted_flag

        if np.linalg.norm(rep_att_vec) > 0:
            rep_att_vec = self.step_noise_size*self.step_snr*rep_att_vec/np.linalg.norm(rep_att_vec)


        while not flag and break_counter < 20:
            break_counter = break_counter + 1
            step = self.step_noise_size * noise_fac * ([0.5, 0.5] - np.random.rand(2)) + rep_att_vec
            if self.is_step_legal(self.current_pos, step, matrix):
                flag = True
                for neighbor_pos in NeighborsPosList:
                    if self.outOfLimit_Ando(neighbor_pos, step):
                        flag = False
                        break
                    if not self.is_step_in_corridor(step, neighbor_pos, matrix):
                        flag = False
                        break

        if break_counter < 20:
            self.next_pos = self.current_pos + step
    # The "is_step_in_corridor" functions let connected agent to disconnect with small probability


    def non_scaned_list(self, p, r, matrix):
        non_sc_list = list()
        base_unit_vector = [[0, 1]]
        for theta in np.linspace(0, 2*np.pi, num=10, endpoint=False):
            unit_vector = [[base_unit_vector[0][0]*np.cos(theta) - base_unit_vector[0][1]*np.sin(theta),
                            base_unit_vector[0][0] * np.sin(theta) + base_unit_vector[0][1] * np.cos(theta)]]
            for dr in np.linspace(self.res, r, num=r / self.res * 2, endpoint=True):
                p2 = p + [[unit_vector[0][0]*dr, unit_vector[0][1]*dr]]
                i, j = self.xy_to_ij(p2[0][0], p2[0][1])
                if 0 <= i and i < matrix.shape[0] and 0 <= j and j < matrix.shape[1]:
                    if matrix[i][j] == 2:
                        break
                    else:
                        if matrix[i][j] == 0:
                            non_sc_list.append([self.ij_to_xy(i, j)])
        return non_sc_list


    def is_step_in_corridor(self, step, neighbor_pos, matrix):
        neighbor_abs_pos = self.current_pos + neighbor_pos
        if self.is_step_legal(neighbor_abs_pos, step, matrix):
            neighbor_abs_pos_potential = neighbor_abs_pos + step
        else:
            neighbor_pos_unit = neighbor_pos / np.linalg.norm(neighbor_pos)
            neighbor_step_potential = step - 2 * np.dot(step[0], neighbor_pos_unit[0]) * neighbor_pos_unit
            neighbor_abs_pos_potential = neighbor_abs_pos + neighbor_step_potential

        return self.is_los(self.current_pos + step, neighbor_abs_pos_potential, matrix)


    def outOfLimit_Ando(self, neighbor_pos, step):
        avg_pos = np.divide(neighbor_pos, 2)
        deltas_step = step - avg_pos
        return np.linalg.norm(deltas_step) > self.VisibilityRange/2


    def Sensing(self, agents_arr, matrix):
        neighbors_pos = []
        for i in range(0, agents_arr.__len__()):
            diff = agents_arr[i].current_pos - self.current_pos
            if (agents_arr[i].ID != self.ID and np.linalg.norm(diff) < self.VisibilityRange and
                    self.is_los(agents_arr[i].current_pos, self.current_pos, matrix)):
                neighbors_pos.append(diff)
        return neighbors_pos


    def neighborhood_reduction(self, neighbors_pos, matrix):
        reduced_neighbors_pos = []
        for i in range(0, neighbors_pos.__len__()):
            flag = True
            counter = 0
            for j in range(0, neighbors_pos.__len__()):
                if i != j and ((np.linalg.norm(neighbors_pos[i]) > np.linalg.norm(neighbors_pos[j])) and
                               (np.linalg.norm(neighbors_pos[i]) >
                                np.linalg.norm(neighbors_pos[j] - neighbors_pos[i]))):
                    if self.is_los(self.current_pos + neighbors_pos[i], self.current_pos + neighbors_pos[j], matrix):
                        counter = counter + 1
                        if counter > 0:
                            flag = False
                            break
            if flag:
                reduced_neighbors_pos.append(neighbors_pos[i])
        return reduced_neighbors_pos


    def update_sate(self, pos, heading):
        self.current_pos = pos
        self.current_heading = heading


    def is_step_legal(self, curr_pos, step, matrix):
        new_pos = curr_pos + step
        i, j = self.xy_to_ij(new_pos[0][0], new_pos[0][1])
        if not (0 <= i and i < matrix.shape[0] and 0 <= j and j < matrix.shape[1] and matrix[i][j] == 0):
            return False
        return self.is_los(curr_pos, new_pos, matrix)


    def xy_to_ij(self, x, y):
        i = int(np.floor((x - self.x_lim[0])/self.res))
        j = int(np.floor((y - self.y_lim[0]) / self.res))
        return i, j

    def ij_to_xy(self, i, j):
        x = self.x_lim[0] + i*self.res + self.res/2
        y = self.y_lim[0] + j*self.res + self.res/2
        return x, y

    def is_los(self, p1, p2, matrix):
        n = int(np.maximum(1, np.ceil(np.linalg.norm(p1-p2)/self.res)*3))
        x = np.linspace(p1[0][0], p2[0][0], num=n, endpoint=True)
        y = np.linspace(p1[0][1], p2[0][1], num=n, endpoint=True)
        for ind in range(1, n):
            i, j = self.xy_to_ij(x[ind], y[ind])
            if matrix[i][j] != 0:
                return False
        return True