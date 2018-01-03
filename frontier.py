import numpy as np
from gridutilities import *

UNKNOWN = 0
EMPTY = 1
OCCUPIED = 2

def getState(point, grid, maxVal = 100):
    """
    Classify a point as occupied, empty or unknown is the grid
    """
    (x,y) = point
    if (grid[x][y] > 0.60*maxVal):
        return OCCUPIED
    elif (grid[x][y] < 0.40*maxVal):
        return EMPTY
    else:
        return UNKNOWN

def isDoable(path, grid, diameter = 1):
    nbErrors = 0
    for (x,y) in path:
        nbOccupied = 0
        neighbors = getAllNeighbors((x,y), grid.shape, diameter = diameter)
        for (xn, yn) in neighbors:
            if (getState((xn,yn), grid) != EMPTY):
                nbOccupied +=1
        if (nbOccupied > 2):
            nbErrors += 1
    return nbErrors <= 5

def biggestFrontier(frontlist):
    return max(frontlist, key=lambda a: len(a[0]))

def isCloseTo(centroid, pointlist, dist = 3.0):
    for (x, y) in pointlist:
        if distance(centroid, (x,y)) <= dist:
            return True
    return False

def filterNotExplored(bound, pointlist, grid, dist = 3):
    filter = []
    for b in bound:
        centroid = computeCentroid(b, grid)
        if not isCloseTo(centroid, pointlist, dist = dist):
            filter.append((b, centroid))
    return filter

def findFrontierPoints(grid):
    """
    Returns all frontier points
    :param grid: probability grid
    :return: list of points such that each point is empty, and has at least one unknown neighbor
    """
    frontier = []
    for i in range(grid.shape[0]):
        for j in range(grid.shape[1]):
            if (getState((i, j), grid) == EMPTY):
                neighbors = getAllNeighbors((i, j), grid.shape)
                isfrontier = False
                for (xn, yn) in neighbors:
                    if getState((xn,yn), grid) == UNKNOWN :
                        isfrontier = True
                if isfrontier:
                    frontier.append((i, j))
    return frontier


def frontierGrid(grid, points):
    """
    Creates a boolean grid where grid[i][j] = true if (i,j) is in the "point" list
    :param grid: grid
    :param points: list of coordinates
    :return:
    """
    fgrid = np.zeros(grid.shape, dtype = bool)
    for (x,y) in points:
        fgrid[x][y] = True
    return fgrid


def findFrontier(point, grid):
    """
    Returns all points in the same frontier as point
    :param point:
    :param grid:
    :return: list of points
    """
    frontier = [point]
    (x,y) = point
    grid[x][y] = False
    neighbors = getAllNeighbors(point, grid.shape, diameter=2)
    for (x,y) in neighbors:
        if grid[x][y] == True:
            #print x, y
            frontier.extend(findFrontier((x,y), grid))
    return frontier


def delimitFrontiers(fgrid):
    """
    Returns a list of frontiers (list of list) from a frontier grid (returned by frontierGrid)
    :param fgrid: boolean grid representing which points are frontier points
    :return: list of frontiers of any size
    """
    frontierlist = []
    for i in range(fgrid.shape[0]):
        for j in range(fgrid.shape[1]):
            if (fgrid[i][j] == True):
                frontierlist.append(findFrontier((i,j), fgrid))
    return frontierlist


def extractFrontiers(grid, minlen = 5):
    """
    Returns a list of all frontiers larger than minlen in a casual grid
    :param grid:
    :param minlen: min length of the frontier in pixel
    :return: a list of list, with each element being a frontier
    """
    points = findFrontierPoints(grid)
    fgrid = frontierGrid(grid, points)
    flist = delimitFrontiers(fgrid)
    flistnew = []
    for list in flist:
        if len(list) >= minlen:
            flistnew.append(list)
    return flistnew


def findNearest(point, grid, state = EMPTY):
    """
    Finds the closest point to point in the grid that is in state "state", and with all its neighbors in this state
    :param point: the point
    :param grid: the grid to search in
    :param state: EMPTY, OCCUPIED or UNKNOWN
    :return:
    """
    tmpgrid = np.zeros(grid.shape, dtype = bool)
    (xp, yp) = point
    neighbors = []
    neighbors.append(point)
    while neighbors:
        (xcur, ycur) = neighbors.pop(0)
        if not tmpgrid[xcur][ycur]:
            if getState((xcur, ycur), grid) == state:
                circle = getAllNeighbors((xcur, ycur), grid.shape, diameter=5)
                empty = True
                for (x, y) in circle:
                    if getState((xcur, ycur), grid) != state:
                        empty = False
                if empty:
                    return (xcur, ycur)
            else:
                neighbors.extend(getAllNeighbors((xcur, ycur), grid.shape))
                tmpgrid[xcur][ycur] = True
    return -1, -1


def computeCentroid(frontier, grid):
    """
    Computes the centroid of a frontier, i.e its barycenter. If centroid is an unknown or occupied area, than we move to the nearest empty point
    :param frontier: list of point delimiting a frontier
    :param grid: the map
    :return: coordinates of the centroid
    """
    xc = 0
    yc = 0
    for (x,y) in frontier:
        xc += x
        yc += y
    n = len(frontier)
    xc = int(xc/n)
    yc = int(yc/n)
    return findNearest((xc, yc), grid, state=EMPTY)

