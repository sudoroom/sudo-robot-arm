#!/usr/bin/env python

from __future__ import division
import math
import sys

"""
Code to generate coordinates for Sudo Room's robot arm

It's a motoman k100s
Natively it measures its joint angles in "pulse counts".
Not sure how to convert those to degrees yet, but this file
uses degrees.

axes:
        S   base yaw -- turn left and right
        L   lower arm pitch -- general tilt up and down
        U   elbow angle
        R   forearm twist -- twists the middle of the upper arm
        B   wrist pitch -- tool tilts up and down (or left and right, depending on R)
        T   finger twist -- spins the tool attachment
"""

#================================================================================
# 3D MATH

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
    """Treat this an an immutable object.
    All methods return new Point objects.
    """
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
    def _applyMatrix(self, matrix):
        result = self.clone()
        vector = [self.x, self.y, self.z]
        result.x = sum([vector[ii] * matrix[0][ii] for ii in range(3)])
        result.y = sum([vector[ii] * matrix[1][ii] for ii in range(3)])
        result.z = sum([vector[ii] * matrix[2][ii] for ii in range(3)])
        return result
    def rotateX(self, degrees, center=None):
        theta = degrees * math.pi / 180
        if not center: center = Point(0,0,0)
        return self.minus(center)._applyMatrix(makeRotateXMatrix(theta)).plus(center)
    def rotateY(self, degrees, center=None):
        theta = degrees * math.pi / 180
        if not center: center = Point(0,0,0)
        return self.minus(center)._applyMatrix(makeRotateYMatrix(theta)).plus(center)
    def rotateZ(self, degrees, center=None):
        theta = degrees * math.pi / 180
        if not center: center = Point(0,0,0)
        return self.minus(center)._applyMatrix(makeRotateZMatrix(theta)).plus(center)
    def distTo(self, other):
        return ((self.x - other.x)**2 + (self.y - other.y)**2 + (self.z - other.z)**2) ** 0.5
    def __repr__(self):
        return '<%9.2f %9.2f %9.2f>' % (self.x, self.y, self.z)
    def asList(self):
        return [self.x, self.y, self.z]


# # example usage:
# p = Point(1,2,3)
# print p
# print p.rotateX(10, Point(0,0,3))
# print p.rotateY(10, Point(0,0,3))
# print p.rotateZ(10, Point(0,0,3))


#================================================================================
# FORWARD KINEMATICS

def getPointFromAngles(angles):
    """Given a list of 6 joint angles in degrees (S, L, U, R, B, T), 
    return the 3d point at the end of the robot arm, in meters.

    If you're sitting "behind" the robot arm so it's reaching away from you,
        x is sideways
        y is up
        z is forwards from the base towards the hand

    I'm not sure which way the rotation goes -- needs testing.

    The "rest position" of the robot in this code, with all angles set to zero:
        Lower arm is vertical
        Forearm is horizontal
        Forearm twist is set so that the wrist moves up and down, not side to side
        The wrist is in line with the forearm
    Don't know yet how this translates to native robot "pulse count" angles.
    """
    S, L, U, R, B, T = angles

    # apply transformations in backwards order, hand first

    # TODO: this gives the point at the end of the "finger", where
    # tools attach.  We also need to take into account the offset from
    # the finger to the end of the tool.


    # measurements from the diagram on page 19 of
    # http://spaz.org/~jake/robot/479951-4-K10S_manual.pdf
    # in that document, units are in tenths of a centimeter (1 meter = 1000 units)
    # here we use meters
    # some measurements are estimated and need to be physically measured

    # [apply tool offset here]

    p = Point(0,0,0)
    p = p.rotateZ(T)                        # finger twist
    # print 'finger twist      ', p
    p = p.translate(Point(0, 0, 0.100))       # wrist -> finger
    # print 'wrist -> finger   ', p
    p = p.rotateX(B)                        # wrist pitch
    # print 'wrist pitch       ', p
    p = p.translate(Point(0, 0, 0.770/2))     # forearm -> wrist
    # print 'forearm -> wrist  ', p
    p = p.rotateZ(R)                        # forearm twist
    # print 'forearm twist     ', p
    p = p.translate(Point(0, 0.077, 0.770/2))    # elbow -> forearm
    # print 'elbow -> forearm  ', p
    p = p.rotateX(U)                        # elbow joint
    # print 'elbow angle       ', p
    p = p.translate(Point(0, 0.500, 0))       # shoulder -> elbow
    # print 'shoulder -> elbow ', p
    p = p.rotateX(L)                        # shoulder joint
    # print 'shoulder angle    ', p
    p = p.translate(Point(0, 0.585, 0.152))     # base -> shoulder
    # print 'base -> shoulder  ', p
    p = p.rotateY(S)                        # base yaw
    # print 'base yaw          ', p
    return p


#================================================================================
# INVERSE KINEMATICS

def generateAllAngles():
    """Yield all legal combinations of angles, in degees, as 6-tuples.
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
                # leave the final joints at zero for now
                R = 0
                B = 0
                T = 0
                yield (S, L, U, R, B, T)

def getAnglesAndDistFromPoint(targetPoint):
    """Search through combinations of angles and return the angles
    that put the robot's finger closest to targetPoint.
    Currently this is a very slow brute force search.
    Returns a 6-tuple of degrees in (S,L,U,R,B,T) order.
    """
    bestDist = 99999999999
    bestAngles = None
    for angles in generateAllAngles():
        thisPoint = getPointFromAngles(angles)
        dist = thisPoint.distTo(targetPoint)
        if dist < bestDist:
            bestDist = dist
            bestAngles = angles
    return (bestAngles, bestDist)


#================================================================================
# MAIN

def helpAndQuit():
    print 'Usage:'
    print '    robot-coords.py --angles 1 2 3 4 5 6      // convert joint angles to 3d point'
    print '    robot-coords.py --point 1 2 3 [--verbose] // convert 3d point to joint angles'
    print 'Units:'
    print '    joint angles: degrees'
    print '    3d point: meters'
    sys.exit(0)

if __name__ == '__main__':

    # process command line arguments
    ARGS = sys.argv[1:]
    if len(ARGS) == 0 or ARGS[0] not in ('--angles', '--point', '--test'):
        helpAndQuit()
    CMD = ARGS[0]
    VERBOSE = '--verbose' in ARGS or '-v' in ARGS
    if VERBOSE:
        ARGS = [arg for arg in ARGS if arg != '--verbose' and arg != '-v']
    COORDS = [float(arg) for arg in ARGS[1:]]

    if CMD == '--angles':
        print getPointFromAngles(COORDS).asList()
    elif CMD == '--point':
        angles, dist = getAnglesAndDistFromPoint(Point(COORDS[0], COORDS[1], COORDS[2]))
        if VERBOSE:
            print 'distance from robot finger to target point: %s' % dist
        print list(angles)
    elif CMD == '--test':
        # this is the rest position and should give all zero angles
        targetPoint = Point(0, 1.162, 1.022)
        print 'target point: %s' % targetPoint
        print 'finding angles to hit that point...'
        angles, dist = getAnglesAndDistFromPoint(targetPoint)
        print 'dist: %s' % dist
        print 'angles: %s' % repr(angles)
    else:
        print 'unknown command'
        helpAndQuit()



