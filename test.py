
import csv
import numpy as np
import matplotlib.pyplot as plt


def xy_to_ij(x, y, x_lim, y_lim, res):
    i = int(np.floor((x - x_lim[0]) / res))
    j = int(np.floor((y - y_lim[0]) / res))
    return i, j


if __name__ == "__main__":

    x_lim = [0, 510]
    y_lim = [0, 490]
    res = 10
    maze_res = 7.6
    maze = []
    grid_maze = np.zeros([np.int64(np.ceil((x_lim[1]-x_lim[0])/res)), np.int64(np.ceil((y_lim[1]-y_lim[0])/res))])
    datafile = open('D:\RafaelDocuments/MazeMap.csv', 'r')
    datareader = csv.reader(datafile, delimiter=';')
    for srow in datareader:
        row = [int(x) for x in srow[0].split(',')]
        maze.append(row)
    maze = np.array(maze)
    maze = np.transpose(maze)
    maze = np.flipud(maze)
    maze = np.fliplr(maze)
    for i_idx in range(np.shape(maze)[0]):
        for j_idx in range(np.shape(maze)[1]):
            if maze[i_idx][j_idx] == 1:
                x_idx = j_idx * maze_res
                y_idx = i_idx * maze_res
                i, j = xy_to_ij(x_idx, y_idx, x_lim, y_lim, res)
                grid_maze[i][j] = 2

    plt.figure(45645)
    plt.imshow(grid_maze, origin='lower')
    plt.show()