"""
Example demonstrating how to communicate with Microsoft Robotic Developer
Studio 4 via the Lokarria http interface. 

Author: Erik Billing (billing@cs.umu.se)

Updated by Ola Ringdahl 2014-09-11
Updated by Ola Ringdahl 2017-10-18 (added example code that use showMap)
"""



import httplib, json, time
import numpy as np
from math import sin, cos, pi, atan2
from show_map import ShowMap
HEADERS = {"Content-type": "application/json", "Accept": "text/json"}

def toWorldCoordinates(xrob, yrob, robotpos, headingangle):
    #pos = self.getPosition()
    #alpha = self.getHeadingAngle()
    xconv = robotpos[0] + xrob * np.sin(headingangle) + yrob * np.cos(headingangle)
    yconv = robotpos[1] + xrob * (-np.cos(headingangle)) + yrob * np.sin(headingangle)
    return xconv, yconv

class UnexpectedResponse(Exception):
    pass

class Robot:
    def __init__(self, url):
        self.MRDS_URL = url
        pass

    def postSpeed(self, angularSpeed, linearSpeed):
        """Sends a speed command to the MRDS server"""
        mrds = httplib.HTTPConnection(self.MRDS_URL)
        params = json.dumps({'TargetAngularSpeed': angularSpeed, 'TargetLinearSpeed': linearSpeed})
        mrds.request('POST', '/lokarria/differentialdrive', params, HEADERS)
        response = mrds.getresponse()
        status = response.status
        # response.close()
        if status == 204:
            return response
        else:
            raise UnexpectedResponse(response)

    def getLaser(self):
        """Requests the current laser scan from the MRDS server and parses it into a dict"""
        mrds = httplib.HTTPConnection(self.MRDS_URL)
        mrds.request('GET', '/lokarria/laser/echoes')
        response = mrds.getresponse()
        if response.status == 200:
            laserData = response.read()
            response.close()
            return json.loads(laserData)
        else:
            return response

    def getLaserAngles(self):
        """Requests the current laser properties from the MRDS server and parses it into a dict"""
        mrds = httplib.HTTPConnection(self.MRDS_URL)
        mrds.request('GET', '/lokarria/laser/properties')
        response = mrds.getresponse()
        if response.status == 200:
            laserData = response.read()
            response.close()
            properties = json.loads(laserData)
            beamCount = int((properties['EndAngle'] - properties['StartAngle']) / properties['AngleIncrement'])
            a = properties['StartAngle']  # +properties['AngleIncrement']
            angles = []
            while a <= properties['EndAngle']:
                angles.append(a)
                a += pi / 180  # properties['AngleIncrement']
            # angles.append(properties['EndAngle']-properties['AngleIncrement']/2)
            return angles
        else:
            raise UnexpectedResponse(response)

    def getPose(self):
        """Reads the current position and orientation from the MRDS"""
        mrds = httplib.HTTPConnection(self.MRDS_URL)
        mrds.request('GET', '/lokarria/localization')
        response = mrds.getresponse()
        if response.status == 200:
            poseData = response.read()
            response.close()
            return json.loads(poseData)
        else:
            return UnexpectedResponse(response)

    def getHeading(self):
        """Returns the XY Orientation as a heading unit vector"""
        return heading(self.getPose()['Pose']['Orientation'])

    def getHeadingAngle(self):
        head = self.getHeading()
        return np.arctan2(head['Y'], head['X'])

    def getPosition(self):
        pose = self.getPose()
        return pose['Pose']['Position']['X'], pose['Pose']['Position']['Y']




def heading(q):
    return rotate(q, {'X': 1.0, 'Y': 0.0, "Z": 0.0})


def rotate(q, v):
    return vector(qmult(qmult(q, quaternion(v)), conjugate(q)))


def quaternion(v):
    q = v.copy()
    q['W'] = 0.0
    return q


def vector(q):
    v = {}
    v["X"] = q["X"]
    v["Y"] = q["Y"]
    v["Z"] = q["Z"]
    return v


def conjugate(q):
    qc = q.copy()
    qc["X"] = -q["X"]
    qc["Y"] = -q["Y"]
    qc["Z"] = -q["Z"]
    return qc


def qmult(q1, q2):
    q = {}
    q["W"] = q1["W"] * q2["W"] - q1["X"] * q2["X"] - q1["Y"] * q2["Y"] - q1["Z"] * q2["Z"]
    q["X"] = q1["W"] * q2["X"] + q1["X"] * q2["W"] + q1["Y"] * q2["Z"] - q1["Z"] * q2["Y"]
    q["Y"] = q1["W"] * q2["Y"] - q1["X"] * q2["Z"] + q1["Y"] * q2["W"] + q1["Z"] * q2["X"]
    q["Z"] = q1["W"] * q2["Z"] + q1["X"] * q2["Y"] - q1["Y"] * q2["X"] + q1["Z"] * q2["W"]
    return q


