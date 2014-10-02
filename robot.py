#!/usr/bin/env python

from __future__ import division
import math

"""
motoman k100s
pulse counts --> degrees?

axes:
        S   base yaw
        L   lower arm pitch
        U   elbow angle
        R   forearm roll
        B   wrist pitch
        T   finger twist
"""

# http://blender.stackexchange.com/questions/8322/understanding-3d-transforms-and-rotations
def makeRotateXMatrix(theta):
    return  [   [1, 0,                0],
                [0, math.cos(theta), -math.sin(theta)],
                [0, math.sin(theta),  math.cos(theta)]]
def makeRotateYMatrix(theta):
    return  [   [ math.cos(theta), 0, math.sin(theta)],
                [ 0,               1, 0              ],
                [-math.sin(theta), 0, math.cos(theta)]]
def makeRotateZMatrix(theta):
    return  [   [math.cos(theta), -math.sin(theta), 0],
                [math.sin(theta),  math.cos(theta), 0],
                [0,                0,               1]]

class Point(object):
    def __init__(self, x, y, z):
        self.x = x
        self.y = y
        self.z = z
    def clone(self):
        return Point(self.x, self.y, self.z)
    def plus(self, other):
        result = self.clone()
        result.x += other.x
        result.y += other.y
        result.z += other.z
        return result
    def translate(self, other):
        return self.plus(other)
    def minus(self, other):
        result = self.clone()
        result.x -= other.x
        result.y -= other.y
        result.z -= other.z
        return result
    def applyMatrix(self, matrix):
        result = self.clone()
        vector = [self.x, self.y, self.z]
        result.x = sum([vector[ii] * matrix[0][ii] for ii in range(3)])
        result.y = sum([vector[ii] * matrix[1][ii] for ii in range(3)])
        result.z = sum([vector[ii] * matrix[2][ii] for ii in range(3)])
        return result
    def rotateX(self, degrees, center=None):
        theta = degrees * math.pi / 180
        if not center: center = Point(0,0,0)
        return self.minus(center).applyMatrix(makeRotateXMatrix(theta)).plus(center)
    def rotateY(self, degrees, center=None):
        theta = degrees * math.pi / 180
        if not center: center = Point(0,0,0)
        return self.minus(center).applyMatrix(makeRotateYMatrix(theta)).plus(center)
    def rotateZ(self, degrees, center=None):
        theta = degrees * math.pi / 180
        if not center: center = Point(0,0,0)
        return self.minus(center).applyMatrix(makeRotateZMatrix(theta)).plus(center)
    def distTo(self, other):
        return ((self.x - other.x)**2 + (self.y - other.y)**2 + (self.z - other.z)**2) ** 0.5
    def __repr__(self):
        return '<%9.2f %9.2f %9.2f>' % (self.x, self.y, self.z)


# p = Point(1,2,3)
# print p
# print p.rotateX(10, Point(0,0,3))
# print p.rotateY(10, Point(0,0,3))
# print p.rotateZ(10, Point(0,0,3))



def getPointFromAngles(angles):
    """angles: a list of 6 angles, in degrees
    x is sideways
    y is up
    z is towards the hand
    """
    S, L, U, R, B, T = angles

    # apply transformations backwards, hand first

    p = Point(0,0,0)
    p = p.rotateZ(T)                        # finger twist
    # print 'finger twist      ', p
    p = p.translate(Point(0, 0, 100))       # wrist -> finger
    # print 'wrist -> finger   ', p
    p = p.rotateX(B)                        # wrist pitch
    # print 'wrist pitch       ', p
    p = p.translate(Point(0, 0, 770/2))     # forearm -> wrist
    # print 'forearm -> wrist  ', p
    p = p.rotateZ(R)                        # forearm twist
    # print 'forearm twist     ', p
    p = p.translate(Point(0, 77, 770/2))    # elbow -> forearm
    # print 'elbow -> forearm  ', p
    p = p.rotateX(U)                        # elbow joint
    # print 'elbow angle       ', p
    p = p.translate(Point(0, 500, 0))       # shoulder -> elbow
    # print 'shoulder -> elbow ', p
    p = p.rotateX(L)                        # shoulder joint
    # print 'shoulder angle    ', p
    p = p.translate(Point(0, 585, 152))     # base -> shoulder
    # print 'base -> shoulder  ', p
    p = p.rotateY(S)                        # base yaw
    # print 'base yaw          ', p
    return p

def generateAllAngles():
    """
        S   base yaw
        L   lower arm pitch
        U   elbow angle
        R   forearm roll
        B   wrist pitch
        T   finger twist
    """
    step = 5
    for S in range(0, 360, step):
        for L in range(-80, 80, step):
            for U in range(-45, 45, step):
                R = 0
                B = 0
                T = 0
                yield (S, L, U, R, B, T)

def getAnglesFromPoint(targetPoint):
    bestDist = 99999999999
    bestAngles = None
    for angles in generateAllAngles():
        thisPoint = getPointFromAngles(angles)
        dist = thisPoint.distTo(targetPoint)
        if dist < bestDist:
            bestDist = dist
            bestAngles = angles
    return bestAngles

# getPointFromAngles((0,0,0,0,0,0))
targetPoint = Point(0, 1162, 1022) # should give all zero angles
print 'target point: %s' % targetPoint
print 'finding angles to hit that point...'
angles = getAnglesFromPoint(targetPoint)
print 'angles: %s' % repr(angles)



