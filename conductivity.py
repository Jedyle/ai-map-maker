from gridutilities import *
import numpy as np

conductivityValues = [int(2 * INFINITE_COND * 1.0 / (1 + np.exp(i*1.0 / 2))) for i in range(10)]

def assignCondValueToNeighbors(point, cspace, value):
    if (value != len(conductivityValues)):
        neighbors = getLineNeighbors(point, cspace.shape)
        for (x, y) in neighbors :
            if cspace[x][y] < conductivityValues[value]:
                cspace[x][y] = conductivityValues[value]
                assignCondValueToNeighbors((x,y), cspace, value+1)

def createGrid(grid):
    """
    Creates a conductivity grid, where cond[i][j] depends on the proximity of (i,j) to an obstacle
    :param grid: probability grid
    :return: a grid of the same size, with conductivity values between INFINITY and 0
    """
    cspace = np.zeros(grid.shape) #(weight, conductivity)
    obstacles = getObstaclePoints(grid)
    for (x, y) in obstacles:
        cspace[x][y] = INFINITE_COND
        assignCondValueToNeighbors((x, y), cspace, 1)
    return cspace
