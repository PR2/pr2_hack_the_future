#!/usr/bin/env python

import roslib
roslib.load_manifest('pr2_simple_interface')
#import rospy

#rospy.init_node('steve_demo')
from pr2_simple_interface import *

start()
#############################################################
# HowTo:
# 
# Look around:
# head.look_at( 1.0, [left+/right-], [ up>1, down <1 ])
#
# Open grippers:
# gripper.rel( [LEFT | RIGHT | BOTH] )
#
# Close grippers
# gripper.close( [LEFT | RIGHT | BOTH] )
#
# Wait for slap on gripper
# gripper.wait_for_slap( [LEFT | RIGHT | BOTH] )
#
# Move torso up/down
# torso.set( [height] )
#
# Move arms
# arm.move_to([-1.42, 0.640, 0.647, -1.925, 30.931, -0.521, -16.642], RIGHT)
#             [ position ], [ LEFT | RIGHT | BOTH] )
# arm.move_to([shoulder_pan,shoulder_life
#
# Speech:
# sound.say("Something")


gripper = Gripper()
arm = RobotArm()
head = Head()
torso = Torso()



# move torso up
torso.set(0.1)

# nod head
head.look_at(1.0, 0.0, 0.5) # look down
head.wait_for()             # wait for head to stop moving
head.look_at(1.0, 0.0, 1.5) # look up
head.wait_for()
head.look_at(1.0, 0.0, 0.5)
head.wait_for()
head.look_at(1.0, 0.0, 1.5)
head.wait_for()
head.look_at(1.0, 0.0, 1.0)
head.wait_for()
torso.set(0.0)

# look straight ahead
head.look_at(1.0, 0.0, 1.0)
head.wait_for()


#arms
arm.move_to([-80, 40, 30, -110, 200, -30, -900], RIGHT)
arm.move_to([80, 40, -30, -110, -200, 30, 900], LEFT)
arm.wait_for(BOTH)

gripper.rel(BOTH)
gripper.wait_for(BOTH)

rospy.sleep(2)


arm.move_to([-20, -15, -20, -50, 60, -5, 700], RIGHT)
arm.move_to([20, -15, 20, -50, -60, 5, -700], LEFT)


gripper.wait_for_slap(BOTH)

head.look_at(1.0, 0.0, 1.5)
head.wait_for()
head.look_at(1.0, 0.0, 1.0)
head.wait_for()

if (gripper.determine_slap() == LEFT):
    head.look_at(1.0, 1.0, 0.5)
    head.wait_for()
else:
    head.look_at(1.0, -1.0, 0.0)
    head.wait_for()

#arms

arm.move_to([70, 50, 40, -120, 100, -20, 20], LEFT)
arm.move_to([-70, 50, -40, -120, -100, -20, -20], RIGHT)
arm.wait_for(BOTH)

gripper.close(BOTH)
gripper.wait_for(BOTH)

head.look_at(1.0, 0.0, 1.0)
head.wait_for()

