from grid import *
from robot import *
from show_map import ShowMap
import time
import threading

def createMap():
    """"A simple example of how to use the ShowMap class"""
    showGUI = True  # set this to False if you run in putty
    # use the same no. of rows and cols in map and grid:

    # create a grid with all cells=7 (unexplored) as numpy matrix:

    robot = Robot()
    origin = (-5.0,-5.0)
    end = (5.0,5.0)
    grid = Grid(origin, end, 20, robot)
    map = ShowMap(grid.dimensions[0], grid.dimensions[1], showGUI)

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
        #time.sleep(5)

if __name__ == '__main__':
    robot = Robot()
    print 'Sending commands to MRDS server', robot.MRDS_URL
    createMap()
