import numpy as np
from robot import *
import drawOnGrid

def distance(a, b):
    return np.sqrt((a[0]-b[0])**2 + (a[1] - b[1])**2)

def calcangle(tuple1, tuple2):
    return np.arctan2(tuple2[1], tuple2[0]) - np.arctan2(tuple1[1], tuple1[0])

class Grid():
    def __init__(self, originpoint, endpoint, pixpermeter, robot):
        self.robot = robot
        self.pixpermeter = pixpermeter #grid resolution
        self.origin = originpoint #lower left point of the map (absolute coordinates)
        self.endpoint = endpoint #upper right point of the map
        self.dimensions = (self.coordinateToGrid(endpoint)[0] - self.coordinateToGrid(originpoint)[0], self.coordinateToGrid(endpoint)[1] - self.coordinateToGrid(originpoint)[1])
        self.grid = np.ones(shape = self.dimensions)*50.0 #matrix with 50 everywhere (unexplored)


        #Data for Bayesian laser model
        self.lobeangle = 1.0*20.0/self.pixpermeter #for scanning
        self.maxGridRange = 20 * self.pixpermeter #max scanning range relative to the grid (in pixels)
        self.region1semibreadth = 0.1 * self.pixpermeter # 1/2 breadth of region I in the laser model
        self.pmax = 0.98 #Pmax in laser model


    def scanArea(self, breadth = 135):
        scan = self.robot.getLaser()['Echoes'][135-breadth:135+breadth]
        robotpos = self.robot.getPosition()
        headingvector = self.robot.getHeading()
        headingangle = self.robot.getHeadingAngle()
        robotcoordinates = (robotpos[0]-self.origin[0], robotpos[1]-self.origin[1]) #robot coordinates in the referential where the origin is at self.origin
        for i in range(len(scan)):
            angle = i - breadth
            s = scan[i]

            # The following calculus give the relative coordinates (robot coord) of the vertices of the triangle we create
            (x1, y1) = (s * (-np.sin((angle + self.lobeangle ) * np.pi / 180)), s * (np.cos((angle + self.lobeangle) * np.pi / 180)))
            (x2, y2) = (s * (-np.sin((angle - self.lobeangle) * np.pi / 180)), s * (np.cos((angle - self.lobeangle) * np.pi / 180)))

            #switch from robot the world coordinate
            (xw1, yw1) = toWorldCoordinates(x1, y1, robotpos, headingangle)
            (xw2, yw2) = toWorldCoordinates(x2, y2, robotpos, headingangle)

            #translation of the referential, so that the origin corresponds to the lower left corn (self.origin)
            v1 = (xw1 - self.origin[0], yw1 - self.origin[1])
            v2 = (xw2 - self.origin[0], yw2 - self.origin[1])

            pointorigin = self.coordinateToGrid(robotcoordinates)
            vg1 = self.coordinateToGrid(v1)
            vg2 = self.coordinateToGrid(v2)

            triangle = drawOnGrid.drawTriangle(pointorigin, vg1, vg2)

            scanToGrid = s * self.pixpermeter #scanToGrid = scanned laser value converted into pixels
            for point in triangle :
                r = distance(point, pointorigin)
                headingtuple = (headingvector['X'], headingvector['Y'])
                vectpoint = (point[0] - pointorigin[0], point[1] - pointorigin[1])
                bearing = calcangle(headingtuple, vectpoint)* 180 / np.pi
                alpha = bearing - angle
                self.assignProba(point, r, alpha, scanToGrid)

    def assignProba(self, point, r, alpha, s):
        if ((point[0] >= self.dimensions[0]) or (point[0] < 0) or (point[1] >= self.dimensions[1]) or (point[1] < 0) or (r > self.maxGridRange) or (abs(alpha) > self.lobeangle)):
            pass
        elif (r > s + self.region1semibreadth): #region III
            pass
        elif (r < s - self.region1semibreadth): #region II
            #print self.maxGridRange, r, self.lobeangle, alpha
            pempty = 0.5 * ( (self.maxGridRange - r)*1.0/self.maxGridRange + (self.lobeangle - abs(alpha))/self.lobeangle )
            pocc = 1 - pempty
            pAnterior = self.grid[point[0]][point[1]]*1.0/100 #prob in this cell before the calculus (stored btw 0 and 100 in the grid)
            pNew = pocc * pAnterior / (pocc * pAnterior + pempty * (1 - pAnterior))
            self.grid[point[0]][point[1]] = 100*pNew
        else: #region I
            pocc = 0.5 * ( (self.maxGridRange - r)*1.0/self.maxGridRange + (self.lobeangle - abs(alpha))/self.lobeangle ) * self.pmax
            pempty = 1 - pocc
            pAnterior = self.grid[point[0]][point[1]]*1.0/100 #prob in this cell before the calculus (stored btw 0 and 100 in the grid)
            pNew = pocc * pAnterior / (pocc * pAnterior + pempty * (1 - pAnterior))
            self.grid[point[0]][point[1]] = 100*pNew

    def coordinateToGrid(self, coord): #transforms a coordinate > 0 into a position in the grid
        return (int(np.floor(coord[0] * self.pixpermeter)), int(np.floor(coord[1] * self.pixpermeter)))


"""
    #old version with prob = 0 or 1 (obstacle or no obstacle)
    def scanArea(self, breadth = 45):
        scan = self.robot.getLaser()['Echoes'][135-breadth:135+breadth]
        print scan
        robotpos = self.robot.getPosition()
        headingvector = self.robot.getHeading()
        headingangle = self.robot.getHeadingAngle()
        origin = (robotpos[0]-self.origin[0], robotpos[1]-self.origin[1])
        for i in range(len(scan)):
            angle = i - breadth
            xdest = scan[i]*(-np.sin(angle * np.pi / 180))
            ydest = scan[i]*(np.cos(angle * np.pi / 180))
            (xworld, yworld) = toWorldCoordinates(xdest, ydest, robotpos, headingangle)
            destination = (xworld - self.origin[0], yworld - self.origin[1])
            pointorigin = self.coordinateToGrid(origin)
            pointdest = self.coordinateToGrid(destination)
            line = drawOnGrid.drawLine(pointorigin, pointdest)
            for point in line:
                if not ((point[0] >= self.dimensions[0]) or (point[0] < 0) or (point[1] >= self.dimensions[1]) or (point[1] < 0)):
                    self.grid[point[0]][point[1]] = 0
            lastpoint = line[len(line)-1]
            if not ((lastpoint[0] >= self.dimensions[0]) or (lastpoint[0] < 0) or (lastpoint[1] >= self.dimensions[1]) or (lastpoint[1] < 0)):
                self.grid[lastpoint[0]][lastpoint[1]] = 100
"""




