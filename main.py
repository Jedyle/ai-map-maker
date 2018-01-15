from grid import *
from wavefront import *
import time
import frontier
import conductivity
from gridutilities import *
import pathTracking as pt
import sys
import threading
import time




def goTo(path, grid, mapGui, url):
    algorithm = pt.PurePursuit(url)
    if (path):
        algorithm.followPath(path)
        grid.robot.postSpeed(0, 0)

def scanAllAround(grid, mapGui):
    grid.robot.postSpeed(0.0, 0.0)
    time.sleep(0.05)
    grid.scanArea()
    grid.robot.postSpeed(2.0, 0.0)
    time.sleep(1.5)
    grid.robot.postSpeed(0.0, 0.0)
    grid.scanArea()
    (robotRow, robotCol) = grid.coordinateToGrid((grid.robot.getPosition()[0], grid.robot.getPosition()[1]))
    mapGui.updateMap(grid.grid, 100, robotRow, robotCol)


def exploreArea(lowLeft, upRight, url):
    showGui = True
    robot = Robot(url = url)

    width = upRight[0] - lowLeft[0] #x
    height = upRight[1] - lowLeft[1] #y

    cspace = Grid(lowLeft, upRight, 200.0/max(width, height), robot)
    map = ShowMap(cspace.dimensions[0], cspace.dimensions[1], showGui)
    mindist = 0.5*max(width, height)/10

    scanAllAround(cspace, map)
    noGoZones = []
    boundaries = frontier.filterNotExplored(frontier.extractFrontiers(cspace.grid, minlen=15), noGoZones, cspace.grid)
    iterationMax = 50
    i = 0
    while boundaries and i < iterationMax: #while not empty
        print i
        (robotRow, robotCol) = cspace.coordinateToGrid((robot.getPosition()[0], robot.getPosition()[1]))
        (front, centroid) = frontier.biggestFrontier(boundaries)
        tries = 0
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
                if distance(robot.getPosition(), cspace.gridToCoordinate(centroid)) >= mindist: #fail
                    tries += 1
            if (distance(robot.getPosition(), cspace.gridToCoordinate(centroid)) <= mindist) or tries >= 3:
                noGoZones.append(centroid)
                outloop = True
        boundaries = frontier.filterNotExplored(frontier.extractFrontiers(cspace.grid, minlen=15), noGoZones, cspace.grid)
        scanAllAround(cspace, map)
        i += 1


if __name__ == '__main__':
    print 'Sending commands to MRDS server'
    args = sys.argv[1:]
    exploreArea((int(args[0]), int(args[1])), (int(args[2]), int(args[3])), args[4])