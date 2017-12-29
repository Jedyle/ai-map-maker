from grid import *
from robot import *
from show_map import ShowMap
import time
import threading

def createMap():
    showGUI = True  # set this to False if you run in putty
    # use the same no. of rows and cols in map and grid:

    robot = Robot()
    origin = (-1.0, -1.0)
    end = (10.0, 10.0)
    grid = Grid(origin, end, 20, robot) #20 pixels per meter
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
