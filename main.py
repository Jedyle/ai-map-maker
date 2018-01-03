from grid import *
from wavefront import *
import time
import frontier
import conductivity
from gridutilities import *
import pathTracking as pt


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

def goTo(path, grid):
    algorithm = pt.PurePursuit()
    if (path):
        print "Point to go : ", path[len(path) - 1]
        print "Current location : ", grid.robot.getPosition()
        print path
        grid.robot.postSpeed(0, 0)
        algorithm.followPath(path)

"""
def goTo(path, grid, mapGui):
    if (path):
        print "Point to go : ", path[len(path)-1]
    print "Current location :", grid.robot.getPosition()
    grid.scanArea()
    (robotRow, robotCol) = grid.coordinateToGrid((grid.robot.getPosition()[0], grid.robot.getPosition()[1]))
    mapGui.updateMap(grid.grid, 100, robotRow, robotCol)
"""

def scanAllAround(grid, mapGui):
    grid.robot.postSpeed(0.0, 0.0)
    time.sleep(0.1)
    #grid.scanArea()
    grid.scanArea()
    #grid.robot.postSpeed(1.0, 0.0)
    #time.sleep(2)
    #grid.robot.postSpeed(0.0, 0.0)
    #grid.scanArea()
    #grid.scanArea()
    (robotRow, robotCol) = grid.coordinateToGrid((grid.robot.getPosition()[0], grid.robot.getPosition()[1]))
    mapGui.updateMap(grid.grid, 100, robotRow, robotCol)


def exploreArea(lowLeft, upRight):
    showGui = True
    robot = Robot()

    width = upRight[0] - lowLeft[0] #x
    height = upRight[1] - lowLeft[1] #y

    print robot.getPosition()
    cspace = Grid(lowLeft, upRight, 200.0/max(width, height), robot)

    map = ShowMap(cspace.dimensions[0], cspace.dimensions[1], showGui)

    mindist = 0.5*max(width, height)/10

    print "Initializing"

    scanAllAround(cspace, map)
    noGoZones = []
    boundaries = frontier.filterNotExplored(frontier.extractFrontiers(cspace.grid, minlen=15), noGoZones, cspace.grid)
    while boundaries: #while not empty
        (robotRow, robotCol) = cspace.coordinateToGrid((robot.getPosition()[0], robot.getPosition()[1]))
        (front, centroid) = frontier.biggestFrontier(boundaries)
        print "Next point", cspace.gridToCoordinate(centroid)
        tries = 0
        outloop = False
        while (not outloop):
            (robotRow, robotCol) = cspace.coordinateToGrid((robot.getPosition()[0], robot.getPosition()[1]))
            condSpace = conductivity.createGrid(cspace.grid)
            wavefront = WaveFront(centroid, condSpace)
            spath = wavefront.shortestPath((robotRow, robotCol))
            if (not spath): #empty
                cspace.grid[centroid[0]][centroid[1]] = 0.9*100
                for (x,y) in getAllNeighbors(centroid, cspace.grid.shape):
                    cspace.grid[x][y] = 0.9*100
                outloop = True
            elif (not frontier.isDoable(spath, cspace.grid)): #if the robot can't follow the path without hitting an obstacle
                cspace.grid[centroid[0]][centroid[1]] = 0.9 * 100
                print "Path not practicable"
                for (x, y) in front:
                    cspace.grid[x][y] = 0.5*100
                outloop = True
            else:
                wcpath = []
                for point in spath:
                    wcpath.append(cspace.gridToCoordinate(point))
                goTo(wcpath, cspace)
                scanAllAround(cspace, map)
                if distance(robot.getPosition(), cspace.gridToCoordinate(centroid)) >= mindist: #fail
                    tries += 1
            if (distance(robot.getPosition(), cspace.gridToCoordinate(centroid)) <= mindist) or tries >= 3:
                noGoZones.append(centroid)
                outloop = True
                print "centroid ", centroid, "no go zone"
            print "Tries : ", tries
        boundaries = frontier.filterNotExplored(frontier.extractFrontiers(cspace.grid, minlen=15), noGoZones, cspace.grid)
        scanAllAround(cspace, map)


if __name__ == '__main__':
    print 'Sending commands to MRDS server'
    exploreArea((-50.0, -50.0), (50.0, 50.0))