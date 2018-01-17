from grid import *
from wavefront import *
import time
import frontier
import conductivity
from gridutilities import *
import pathTracking as pt
import sys
from threading import Thread
import time

def scanContinuously(grid, algo, mapGui):
    while not algo.finished :
        grid.scanArea(breadth=30)
        (robotRow, robotCol) = grid.coordinateToGrid((grid.robot.getPosition()[0], grid.robot.getPosition()[1]))
        mapGui.updateMap(grid.grid, 100, robotRow, robotCol)
        #time.sleep(0.5)


def goTo(path, grid, mapGui, url):
    algorithm = pt.PurePursuit(url)
    if (path):
        grid.robot.postSpeed(0, 0)
        scan = Thread(target= scanContinuously, args=(grid, algorithm, mapGui))
        pathtracking = Thread(target= algorithm.followPath, args=(path,))
        scan.start()
        pathtracking.start()
        scan.join()
        pathtracking.join()
        grid.robot.postSpeed(0, 0)


def scanAllAround(grid, mapGui):
    grid.robot.postSpeed(0.0, 0.0)
    time.sleep(0.05)
    grid.scanArea(breadth=135)
    grid.robot.postSpeed(2.0, 0.0)
    time.sleep(1.5)
    grid.robot.postSpeed(0.0, 0.0)
    grid.scanArea(breadth=135)
    (robotRow, robotCol) = grid.coordinateToGrid((grid.robot.getPosition()[0], grid.robot.getPosition()[1]))
    mapGui.updateMap(grid.grid, 100, robotRow, robotCol)

def exploreArea(lowLeft, upRight, url):
    start = time.time()
    showGui = True
    robot = Robot(url = url)

    width = upRight[0] - lowLeft[0] #x
    height = upRight[1] - lowLeft[1] #y

    resolution = 300.0

    cspace = Grid(lowLeft, upRight, resolution/max(width, height), robot)
    map = ShowMap(cspace.dimensions[0], cspace.dimensions[1], showGui)

    visitedGrid = np.zeros(cspace.grid.shape, dtype=bool)

    mindist = 0.5*max(width, height)/10
    noGoZones = []
    scanAllAround(cspace, map)
    (robotRow, robotCol) = cspace.coordinateToGrid((robot.getPosition()[0], robot.getPosition()[1]))
    boundaries = frontier.sortFrontiers(frontier.extractFrontiers(cspace.grid, minlen=50, dist = 1), (robotRow, robotCol), cspace.grid, visitedGrid)
    iterationMax = 100
    i = 0
    num = 0
    while boundaries and i < iterationMax and num < cspace.grid.shape[0]*cspace.grid.shape[1]*0.8: #while not empty
        print "No Go : ", noGoZones
        spath = []
        centroid = (-1,-1)

        while (len(spath) == 0 and len(boundaries) > 0):
            print "Computing..."
            front = boundaries.pop(0)
            print "Len front : ", len(front)
            centroid = frontier.computeCentroid(front, cspace.grid)
            condSpace = conductivity.createGrid(cspace.grid)
            wavefront = WaveFront(centroid, (robotRow, robotCol), condSpace)
            spath = wavefront.shortestPath()

        wcpath = []
        for point in spath:
            wcpath.append(cspace.gridToCoordinate(point))
        if (len(wcpath) != 0):
            print "Go to : ", wcpath[len(wcpath) - 1]
        goTo(wcpath, cspace, map, url)
        (robotRow, robotCol) = cspace.coordinateToGrid((robot.getPosition()[0], robot.getPosition()[1]))
        labelVisited(visitedGrid, (robotRow, robotCol), diameter=int(resolution / 25))
        if (distance((robotRow, robotCol), centroid) > 1.0*cspace.pixpermeter):
            labelVisited(visitedGrid, centroid, diameter=int(resolution / 25))
        boundaries = frontier.sortFrontiers(frontier.extractFrontiers(cspace.grid, minlen=50, dist = 2), (robotRow, robotCol), cspace.grid, visitedGrid)
        scanAllAround(cspace, map)
        print "Bound ", boundaries
        print "i ", i
        i += 1
        num = getNumberVisited(visitedGrid)
        print "NbVisited ", num, " ", cspace.grid.shape[0]*cspace.grid.shape[1], " ", num/cspace.grid.shape[0]*cspace.grid.shape[1]
        print "Elapsed : ", time.time() - start

"""
def exploreArea(lowLeft, upRight, url):
    showGui = True
    robot = Robot(url = url)

    width = upRight[0] - lowLeft[0] #x
    height = upRight[1] - lowLeft[1] #y

    cspace = Grid(lowLeft, upRight, 300.0/max(width, height), robot)
    map = ShowMap(cspace.dimensions[0], cspace.dimensions[1], showGui)
    mindist = 0.5*max(width, height)/10

    scanAllAround(cspace, map)
    noGoZones = []
    #boundaries = frontier.filterNotExplored(frontier.extractFrontiers(cspace.grid, minlen=15), noGoZones, cspace.grid)
    boundaries = frontier.extractFrontiers(cspace.grid, minlen=15)
    iterationMax = 50
    i = 0
    while boundaries and i < iterationMax: #while not empty
        print i
        (robotRow, robotCol) = cspace.coordinateToGrid((robot.getPosition()[0], robot.getPosition()[1]))
        (front, centroid) = frontier.biggestFrontier(boundaries)
        #tries = 0
        outloop = False
        while (not outloop):
            (robotRow, robotCol) = cspace.coordinateToGrid((robot.getPosition()[0], robot.getPosition()[1]))
            condSpace = conductivity.createGrid(cspace.grid)
            wavefront = WaveFront(centroid, (robotRow, robotCol), condSpace)
            spath = wavefront.shortestPath()
            if (not spath): #empty
                cspace.grid[centroid[0]][centroid[1]] = 0.9*100
                for (x,y) in getAllNeighbors(centroid, cspace.grid.shape):
                    cspace.grid[x][y] = 0.9*100
                outloop = True
            elif (not frontier.isDoable(spath, cspace.grid)): #if the robot can't follow the path without hitting an obstacle
                cspace.grid[centroid[0]][centroid[1]] = 0.9 * 100
                for (x, y) in front:
                    cspace.grid[x][y] = 0.5*100
                outloop = True
            else:
                wcpath = []
                for point in spath:
                    wcpath.append(cspace.gridToCoordinate(point))
                goTo(wcpath, cspace, map, url)
                scanAllAround(cspace, map)
                #if distance(robot.getPosition(), cspace.gridToCoordinate(centroid)) >= mindist: #fail
                #    tries += 1
            #if (distance(robot.getPosition(), cspace.gridToCoordinate(centroid)) <= mindist) or tries >= 3:
            #    noGoZones.append(centroid)
            #    outloop = True
        #boundaries = frontier.filterNotExplored(frontier.extractFrontiers(cspace.grid, minlen=15), noGoZones, cspace.grid)
        boundaries = frontier.extractFrontiers(cspace.grid, minlen=15)
        scanAllAround(cspace, map)
        i += 1
"""

if __name__ == '__main__':
    print 'Sending commands to MRDS server'
    args = sys.argv[1:]
    exploreArea((int(args[0]), int(args[1])), (int(args[2]), int(args[3])), args[4])