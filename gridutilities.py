import numpy as np

INFINITE_COND = 1000
OBSTACLE_VAL = -1
WAVEGRID_INFTY = 100000

def distance(a, b):
    return np.sqrt((a[0]-b[0])**2 + (a[1] - b[1])**2)

def calcangle(tuple1, tuple2):
    return np.arctan2(tuple2[1], tuple2[0]) - np.arctan2(tuple1[1], tuple1[0])


def getAllNeighbors(point, shape, diameter = 1):
    (row, col) = shape
    (x, y) = point
    neighbors = []
    for i in range(x-diameter, x+diameter+1):
        for j in range(y-diameter, y+diameter+1):
            if (i >=0 and i < row and j >= 0 and j < col and (i,j) != (x,y)):
                neighbors.append((i,j))
    return neighbors

def getLineNeighbors(point, shape):  # get row and column neighbors of a point
    (x, y) = point
    neighbors = []
    if (x - 1 >= 0):
        neighbors.append((x - 1, y))
    if (y - 1 >= 0):
        neighbors.append((x, y - 1))
    if (x + 1 < shape[0]):
        neighbors.append((x + 1, y))
    if (y + 1 < shape[1]):
        neighbors.append((x, y + 1))
    return neighbors

def positiveOrInfinity(number):
    if (number <= 0):
        return WAVEGRID_INFTY
    else :
        return number


def getObstaclePoints(grid, limitval=45.0):
    (row, col) = grid.shape
    obstacles = []
    for i in range(row):
        for j in range(col):
            if grid[i][j] > limitval:
                obstacles.append((i, j))
    return obstacles

print getAllNeighbors((0,0), (5,5))
print getAllNeighbors((4,4), (5,5))
print getAllNeighbors((1,1), (5,5))
print getAllNeighbors((2,0), (5,5))
print getAllNeighbors((9,0), (5,5))