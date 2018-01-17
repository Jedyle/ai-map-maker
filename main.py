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
        grid.scanArea(breadth=90)
        (robotRow, robotCol) = grid.coordinateToGrid((grid.robot.getPosition()[0], grid.robot.getPosition()[1]))
        mapGui.updateMap(grid.grid, 100, robotRow, robotCol)

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
    (robotRow, robotCol) = grid.coordinateToGrid((grid.robot.getPosition()[0], grid.robot.getPosition()[1]))
    mapGui.updateMap(grid.grid, 100, robotRow, robotCol)
    grid.robot.postSpeed(2.0, 0.0)
    time.sleep(1.5)
    grid.robot.postSpeed(0.0, 0.0)
    grid.scanArea(breadth=135)
    (robotRow, robotCol) = grid.coordinateToGrid((grid.robot.getPosition()[0], grid.robot.getPosition()[1]))
    mapGui.updateMap(grid.grid, 100, robotRow, robotCol)

def exploreArea(lowLeft, upRight, url):
    start = time.time()
    showGui = True
    robot = Robot(url)
    width = upRight[0] - lowLeft[0] #x
    height = upRight[1] - lowLeft[1] #y
    cspace = Grid(lowLeft, upRight, 3, robot) #3 pixels per meter
    gridResolution = min(cspace.grid.shape[0], cspace.grid.shape[1])
    map = ShowMap(cspace.dimensions[0], cspace.dimensions[1], showGui)
    visitedGrid = np.zeros(cspace.grid.shape, dtype=bool)
    scanAllAround(cspace, map)
    (robotRow, robotCol) = cspace.coordinateToGrid((robot.getPosition()[0], robot.getPosition()[1]))
    frontiers = frontier.extractFrontiers(cspace.grid, minlen=gridResolution/7, dist = 1)
    boundaries = frontier.sortFrontiers(frontiers, (robotRow, robotCol), cspace.grid, visitedGrid)
    iterationMax = int((max(width, height)/10.0)**2)
    i = 0
    num = 0
    while boundaries and i < iterationMax: #while not empty
        spath = []
        centroid = (-1, -1)
        while (len(spath) == 0 and len(boundaries) > 0):
            front = boundaries.pop(0)
            centroid = frontier.computeCentroid(front, cspace.grid)
            condSpace = conductivity.createGrid(cspace.grid)
            wavefront = WaveFront(centroid, (robotRow, robotCol), condSpace)
            spath = wavefront.shortestPath()

        wcpath = []
        for point in spath:
            wcpath.append(cspace.gridToCoordinate(point))
        goTo(wcpath, cspace, map, url)
        (robotRow, robotCol) = cspace.coordinateToGrid((robot.getPosition()[0], robot.getPosition()[1]))
        labelVisited(visitedGrid, (robotRow, robotCol), diameter=int(gridResolution / 25))
        if (distance((robotRow, robotCol), centroid) > 1.0*cspace.pixpermeter):
            labelVisited(visitedGrid, centroid, diameter=int(gridResolution / 25))

        frontiers = frontier.extractFrontiers(cspace.grid, gridResolution / 7, dist=2)
        boundaries = frontier.sortFrontiers(frontiers, (robotRow, robotCol), cspace.grid, visitedGrid)
        scanAllAround(cspace, map)
        i += 1
        num = getNumberVisited(visitedGrid)
        print "Elapsed time : ", time.time() - start

if __name__ == '__main__':
    print 'Starting Robot'
    args = sys.argv[1:]
    exploreArea((int(args[1]), int(args[2])), (int(args[3]), int(args[4])), args[0])