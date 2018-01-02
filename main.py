from grid import *
from wavefront import *
import time
import frontier
import conductivity
from gridutilities import *


"""
def createMap():
    showGUI = True  # set this to False if you run in putty
    # use the same no. of rows and cols in map and grid:

    np.set_printoptions(precision=2)
    robot = Robot()
    origin = (-1.0, -1.0)
    end = (10.0, 10.0)
    grid = Grid(origin, end, 10, robot) #20 pixels per meter
    map = ShowMap(grid.dimensions[0], grid.dimensions[1], showGUI)

    wavemap = ShowMap(grid.dimensions[0], grid.dimensions[1], showGUI, mapName= "conductivity.png")

    # Max grid value
    maxVal = 100

    start_time = time.time()
    while(True):
        # Position of the robot in the grid (red dot)
        (robotRow, robotCol) = grid.coordinateToGrid((robot.getPosition()[0] - origin[0], robot.getPosition()[1] - origin[1]))

        # Update the grid
        grid.scanArea()
        map.updateMap(grid.grid, maxVal, robotRow, robotCol)
        print("Map updated")

        frontiergrid = np.zeros(grid.grid.shape)
        flists = frontier.extractFrontiers(grid.grid, minlen=10)
        for bound in flists:
            for (x,y) in bound:
                frontiergrid[x][y] = 50
            (xc, yc) = frontier.computeCentroid(bound, grid.grid)
            frontiergrid[xc][yc] = 100

        #frontiermap.updateMap(frontiergrid, 100, 0, 0)

        gridbis = grid.grid.copy()
        cspace = conductivity.createGrid(gridbis)
        point = (24, 40)
        wavefront = WaveFront(point, cspace)
        spath = wavefront.shortestPath((robotRow,robotCol))
        for (x, y) in spath:
            gridbis[x][y] = 100
        gridbis[point[0]][point[1]] = 100
        wavemap.updateMap(gridbis, 100, robotRow, robotCol)
        print("Cond map updated")
"""


def goTo(path, grid, mapGui):
    if (path):
        print "Point to go : ", path[len(path)-1]
    print "Current location :", grid.robot.getPosition()
    grid.scanArea()
    (robotRow, robotCol) = grid.coordinateToGrid((grid.robot.getPosition()[0], grid.robot.getPosition()[1]))
    mapGui.updateMap(grid.grid, 100, robotRow, robotCol)


def exploreArea(lowLeft, upRight):
    showGui = True
    robot = Robot()
    cspace = Grid(lowLeft, upRight, 10, robot)

    map = ShowMap(cspace.dimensions[0], cspace.dimensions[1], showGui)
    cspace.scanArea()

    print "Initializing"

    boundaries = frontier.extractFrontiers(cspace.grid, minlen = 10)
    while boundaries: #while not empty
        (robotRow, robotCol) = cspace.coordinateToGrid((robot.getPosition()[0], robot.getPosition()[1]))
        front = frontier.biggestFrontier(boundaries)
        centroid = frontier.computeCentroid(front, cspace.grid)
        print "Next point", cspace.gridToCoordinate(centroid)
        while(distance(robot.getPosition(), cspace.gridToCoordinate(centroid)) > 1.0):
            if (frontier.getState(centroid, cspace.grid) != frontier.EMPTY):
                centroid = frontier.computeCentroid(front, cspace.grid)
            condSpace = conductivity.createGrid(cspace.grid)
            wavefront = WaveFront(centroid, condSpace)
            spath = wavefront.shortestPath((robotRow, robotCol))
            wcpath = []
            for point in spath:
                wcpath.append(cspace.gridToCoordinate(point))
            goTo(wcpath, cspace, map) #coucou dorian c'est pour toi ici :)
        boundaries = frontier.extractFrontiers(cspace.grid, minlen=20)
        cspace.scanArea()


if __name__ == '__main__':
    print 'Sending commands to MRDS server'
    exploreArea((-1.0, -1.0), (10.0, 10.0))