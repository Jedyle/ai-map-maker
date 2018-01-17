import numpy as np
from robot import *
from gridutilities import *

class WaveFront:

    def __init__(self, goalpoint, startpoint, cspace):
        self.i = 0
        self.cspace = cspace
        self.goal = goalpoint
        (xs, ys) = startpoint
        xs = max(0, min(xs, self.cspace.shape[0]-1))
        ys = max(0, min(ys, self.cspace.shape[1]-1))
        self.start = (xs, ys)
        self.wavegrid = self.createWaveGrid()

    def getNeighborsToUpdate(self, grid, point):
        (xp, yp) = point
        gridneighbors = getAllNeighbors(point, grid.shape)
        neighborsToChange = []
        for (x, y) in gridneighbors:
            if ((grid[x][y] == 0 and self.cspace[x][y] != INFINITE_COND) or (grid[xp][yp] + self.cspace[x][y] + 1 < grid[x][y])):
                #if neighbors' weight is null (not visited yet) or its weight is greater than current point's weight + conductivity (if so, udpate it)
                neighborsToChange.append((point,(x,y)))
            if (self.cspace[x][y] == INFINITE_COND):
                grid[x][y] = OBSTACLE_VAL
        return neighborsToChange

    def spreadWave(self, grid, goal, start): #BFS algorithm to spread the wave in the wavegrid
        neighbors = self.getNeighborsToUpdate(grid, goal)
        while(neighbors):
            (goal, point) = neighbors.pop(0)
            (xg, yg) = goal
            (x,y) = point
            if ((grid[xg][yg] + (self.cspace[x][y]+1)*distance((x,y), (xg,yg)) < grid[x][y]) or (abs(grid[x][y]) <= 10**(-4))):
                grid[x][y] = grid[xg][yg] + (self.cspace[x][y]+1)*distance((x, y), (xg, yg))
                neighbors.extend(self.getNeighborsToUpdate(grid, point))
            if goal == start:
                return


    def createWaveGrid(self):
        grid = np.zeros(self.cspace.shape, dtype=float)
        grid[self.goal[0]][self.goal[1]] = 1.0
        self.spreadWave(grid, self.goal, self.start)
        return grid


    def shortestPath(self):
        (xs, ys) = self.start
        current = self.start
        path = [current]
        if (self.wavegrid[xs][ys] <= 0) or (self.wavegrid[self.goal[0]][self.goal[1]] <= 0):
            return []
        else:
            while(current != self.goal):
                neigh = getAllNeighbors(current, self.wavegrid.shape)
                if (len(neigh) != 0):
                    current = min(neigh, key=lambda (x, y): positiveOrInfinity(self.wavegrid[x][y]))
                    path.append(current)
        return path

