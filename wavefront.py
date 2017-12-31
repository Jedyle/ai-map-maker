import numpy as np
from robot import *
from gridutilities import *

class WaveFront:

    def __init__(self, goalpoint, cspace):
        self.i = 0
        self.cspace = cspace
        self.goal = goalpoint
        self.wavegrid = self.createWaveGrid(goalpoint)
        #print self.wavegrid

    def getNeighborsToUpdate(self, grid, point):
        (xp, yp) = point
        gridneighbors = getLineNeighbors(point, grid.shape)
        neighborsToChange = []
        for (x, y) in gridneighbors:
            if ((grid[x][y] == 0 and self.cspace[x][y] != INFINITE_COND) or (grid[xp][yp] + self.cspace[x][y] + 1 < grid[x][y])):
                #if neighbors' weight is null (not visited yet) or its weight is greater than current point's weight + conductivity (if so, udpate it)
                neighborsToChange.append((point,(x,y)))
            if (self.cspace[x][y] == INFINITE_COND):
                grid[x][y] = OBSTACLE_VAL
        return neighborsToChange

    def spreadWave(self, grid, goal): #BFS algorithm to spread the wave in the wavegrid
        (xg, yg) = goal
        neighbors = self.getNeighborsToUpdate(grid, goal)
        while(neighbors):
            (goal, point) = neighbors.pop(0)
            (xg, yg) = goal
            (x,y) = point
            if ((grid[xg][yg] + self.cspace[x][y] + 1 < grid[x][y]) or (grid[x][y] == 0)):
                grid[x][y] = grid[xg][yg] + int(self.cspace[x][y]) + 1
                neighbors.extend(self.getNeighborsToUpdate(grid, point))
        return neighbors


    def createWaveGrid(self, goal):
        grid = np.zeros(self.cspace.shape, dtype=int)
        grid[goal[0]][goal[1]] = 1
        self.spreadWave(grid, goal)
        return grid


    def shortestPath(self, start):
        (xs, ys) = start

        current = start
        path = [current]
        if (self.wavegrid[xs][ys] <= 0):
            print "No path from ", start, " to ", self.goal
            return []
        else:
            while(current != self.goal):
                current = min(getLineNeighbors(current, self.wavegrid.shape), key=lambda (x,y): positiveOrInfinity(self.wavegrid[x][y]))
                path.append(current)
        return path

"""
cspace = np.zeros((10,10))
cspace[9][8] = INFINITE_COND
cspace[8][9] = 15
cspace[8][8] = 10
cspace[0][5] = INFINITE_COND
cspace[1][5] = INFINITE_COND
cspace[2][5] = INFINITE_COND
cspace[3][5] = INFINITE_COND
cspace[3][6] = INFINITE_COND
cspace[3][7] = INFINITE_COND
cspace[2][7] = INFINITE_COND
cspace[1][7] = INFINITE_COND
cspace[0][7] = INFINITE_COND

cspace[0][4] = 1
cspace[1][4] = 1
cspace[2][4] = 1
cspace[3][4] = 1
cspace[4][4] = 1
cspace[4][5] = 1
cspace[4][6] = 1
cspace[4][7] = 1

wave = WaveFront((0,0), cspace)
print wave.cspace
print wave.wavegrid
print wave.shortestPath((0,9))
print wave.shortestPath((0,6))

"""