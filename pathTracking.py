import math
import time
from robot import *
from gridutilities import *
import random

"""
Represents a 2D point
"""


class Point:
    def __init__(self, x, y):
        self.x = x
        self.y = y

    """
    Calculates the distance from this point to the supplied point
    """

    def dist(self, point):
        return math.sqrt((self.x - point.x) ** 2 + (self.y - point.y) ** 2)

    def __str__(self):
        return "X: %.3f, Y: %.3f" % (self.x, self.y)

class PathFollowAlgorithm(object):
    def __init__(self, url):
        self.position = (0, 0)
        self.lookahead_dist = 0.8
        self.pointIndex = 0
        self.robot = Robot(url = url)
        self.collision_dist = 2.0
        self.finished = False
        self.tries = 0
        self.pointToAvoid = 0

    """
    Updates the position of the robot
    """


    def is_collision(self):
        breadth = 20
        echoes = self.robot.getLaser()['Echoes'][135-breadth:135+breadth]
        for signal in echoes :
            if signal < self.collision_dist:
                return True
        return False

    def avoidObstacle(self, breadth = 80, nbzones = 5, distmin = 5):
        self.tries += 1
        print "Tries ", self.tries
        if (self.tries > 10):
            self.finished = True
            self.robot.postSpeed(0,0)
            return
        echoes = self.robot.getLaser()['Echoes'][135-breadth:135+breadth]
        free = [40.0 for i in range(nbzones)]
        for i in range(nbzones):
            for sensor in range(i*int((breadth*2)/nbzones), (i+1)*int((breadth*2)/nbzones)):
                free[i] = min(free[i], echoes[sensor])
        best = free.index(max(free))
        if free[best] >= distmin:
            angle = -breadth + int(breadth/nbzones) + best*int((breadth*2)/nbzones)
            self.robot.postSpeed(angle, 0)
            time.sleep(1)
            self.robot.postSpeed(0, 1)
            time.sleep(1)
            self.robot.postSpeed(0, 0)
        else:
            self.robot.postSpeed(2.0, 0)
            time.sleep(1)
            self.robot.postSpeed(0,0)
            self.avoidObstacle()
            #self.finished = True


    def updatePosition(self):
        position = self.robot.getPose()['Pose']['Position']
        self.position = (position['X'], position['Y'])

    """
    Calculates the next lookahead point on the path
    """

    def getNextPoint(self):
        for i in range(self.pointIndex, len(self.path) - 1):
            if self.lookahead_dist <= distance(self.path[i], self.position):
                self.pointIndex = i
                return self.path[i]
        self.pointIndex = len(self.path) - 1
        return self.path[self.pointIndex]

    """
    Returns true if the robot reached the goal
    """

    def isArrivedToPoint(self):
        dist = distance(self.path[0], self.path[-1])
        #print dist
        return self.pointIndex == len(self.path) - 1 and distance(self.position, (self.path[len(self.path) - 1])) <= dist*0.1

    """
    Follows the path which is specified in the file with name: <file_name>
    """

    def followPath(self, path):
        print path
        self.path = path
        self.pointIndex = 0
        self.run()

    """
    Calculates the error angle to the next point
    """

    def calcAngle(self, point):
        dx = point[0] - self.position[0]
        dy = point[1] - self.position[1]
        # angle to the point
        error_angle = math.atan2(dy, dx)
        heading = self.robot.getHeading()
        # angle to the heading of the robot
        heading_angle = math.atan2(heading['Y'], heading['X'])
        angle = error_angle - heading_angle
        # normalize the angle to be between -pi and pi
        if abs(angle) > math.pi:
            angle -= (angle / abs(angle)) * 2 * math.pi
        return angle

    """
    Runs the core of the algorithm
    """

    def run(self):
        pass


"""
Algorithm that steers the vehicle in a curve to the next point
"""


class PurePursuit(PathFollowAlgorithm):
    """
    Transforms the given point from WCS to the RCS of the robot based on the current position
    """

    def toVehicleCoordinates(self, point):
        dx = point[0] - self.position[0]
        dy = point[1] - self.position[1]
        heading = self.robot.getHeading()
        heading_angle = math.atan2(heading['Y'], heading['X'])
        return (dx * math.cos(heading_angle) + dy * math.sin(heading_angle),
                     - dx * math.sin(heading_angle) + dy * math.cos(heading_angle))

    """
    Runs the core algorithm
    """

    def run(self):
        self.lookahead_dist = 5.0 #0.8
        linear_speed = 4.0 #1.2
        self.robot.postSpeed(0, 0)
        while (not self.isArrivedToPoint()) and (not self.finished):
            if self.is_collision():
                self.robot.postSpeed(0.0, 0.0)
                self.avoidObstacle()
            self.updatePosition()
            # calculate the next goal point based on the lookahead distance
            next_point = self.getNextPoint()
            # if the angle is greater than 90 degrees, we will just turn around without linear movement
            angle = self.calcAngle(next_point)
            if (abs(angle) > math.pi / 2):
                self.robot.postSpeed(angle, 0)
                continue
            # transform the goal point to RCS
            goal_point = self.toVehicleCoordinates(next_point)
            # Calculate the radius
            dist_to_goal = distance(goal_point, (0, 0))
            radius = (dist_to_goal) ** 2 / (2 * goal_point[1])
            # Update the speed of the robot
            angular_speed = (linear_speed / radius)
            self.robot.postSpeed(angular_speed, linear_speed)
        self.robot.postSpeed(0.0, 0.0)
        self.finished = True