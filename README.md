sudo-robot-arm
==============

Some tools for working with Sudo Room's [robot arm](https://sudoroom.org/wiki/Giant_robot_arm).



robot-coords.py
---------------

Converts back and forth between robot joint angles and the 3d point at the end of the arm.

```
Usage:
    robot-coords.py --angles 1 2 3 4 5 6      // convert joint angles to 3d point
    robot-coords.py --point 1 2 3 [--verbose] // convert 3d point to joint angles
Units:
    joint angles: degrees
    3d point: meters
```


sudo-robot-01.blend
-------------------

A blender file with an articulated 3d model of the robot.  The joints are represented by empty objects that you can rotate.

Also an OBJ export of the model.

Note that it uses different units than robot-coods.py -- it's in tenths of a centimeter, and will need to be scaled down by 1000 to be in meters.
