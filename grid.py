import numpy as np
from lokarriaexample import *

class Grid():
    def __init__(self, originpoint, endpoint, pixpermeter, robot):
        self.robot = robot
        self.pixpermeter = pixpermeter
        self.origin = originpoint
        self.endpoint = endpoint
        self.dimensions = (self.coordinateToGrid(endpoint)[0] - self.coordinateToGrid(originpoint)[0], self.coordinateToGrid(endpoint)[1] - self.coordinateToGrid(originpoint)[1])
        #print "Dimensions", self.dimensions
        self.grid = np.ones(shape = self.dimensions)*100 #matrix with 100 everywhere (unexplored)

    def scanArea(self, breadth = 45):
        scan = self.robot.getLaser()['Echoes'][135-breadth:135+breadth]
        print scan
        robotpos = self.robot.getPosition()
        headingangle = self.robot.getHeadingAngle()
        origin = (robotpos[0]-self.origin[0], robotpos[1]-self.origin[1])
        for i in range(len(scan)):
            angle = i - breadth
            print "angle ", angle
            xdest = scan[i]*(-np.sin(angle * np.pi / 180))
            ydest = scan[i]*(np.cos(angle * np.pi / 180))
            print "xydest ", xdest, " ", ydest

            (xworld, yworld) = toWorldCoordinates(xdest, ydest, robotpos, headingangle)
            destination = (xworld - self.origin[0], yworld - self.origin[1])
            print "dest ", destination
            pointorigin = self.coordinateToGrid(origin)
            pointdest = self.coordinateToGrid(destination)
            print "points : ", pointorigin, " " , pointdest
            line = self.drawLine(pointorigin, pointdest)
            for point in line:
                if not ((point[0] >= self.dimensions[0]) or (point[0] < 0) or (point[1] >= self.dimensions[1]) or (point[1] < 0)):
                    self.grid[point[0]][point[1]] = 0
            lastpoint = line[len(line)-1]
            if not ((lastpoint[0] >= self.dimensions[0]) or (lastpoint[0] < 0) or (lastpoint[1] >= self.dimensions[1]) or (lastpoint[1] < 0)):
                self.grid[lastpoint[0]][lastpoint[1]] = 100

    def coordinateToGrid(self, coord): #transforms a coordinate > 0 into a position in the grid
        print "coord grid ", coord
        return (int(np.floor(coord[0] * self.pixpermeter)), int(np.floor(coord[1] * self.pixpermeter)))

    def drawLine(self, start, end):
        """Bresenham's Line Algorithm
        Produces a list of tuples from start and end
        Source : roguebasin.com
        """

        # Setup initial conditions
        x1, y1 = start
        x2, y2 = end
        dx = x2 - x1
        dy = y2 - y1

        # Determine how steep the line is
        is_steep = abs(dy) > abs(dx)

        # Rotate line
        if is_steep:
            x1, y1 = y1, x1
            x2, y2 = y2, x2

        # Swap start and end points if necessary and store swap state
        swapped = False
        if x1 > x2:
            x1, x2 = x2, x1
            y1, y2 = y2, y1
            swapped = True

        # Recalculate differentials
        dx = x2 - x1
        dy = y2 - y1

        # Calculate error
        error = int(dx / 2.0)
        ystep = 1 if y1 < y2 else -1

        # Iterate over bounding box generating points between start and end
        y = y1
        points = []
        for x in range(x1, x2 + 1):
            coord = (y, x) if is_steep else (x, y)
            points.append(coord)
            error -= abs(dy)
            if error < 0:
                y += ystep
                error += dx

        # Reverse the list if the coordinates were swapped
        if swapped:
            points.reverse()
        return points
