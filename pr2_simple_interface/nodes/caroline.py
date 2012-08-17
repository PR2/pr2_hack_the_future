#!/usr/bin/env python

import roslib
roslib.load_manifest('pr2_simple_interface')
roslib.load_manifest('sound_play')
#import rospy

#rospy.init_node('steve_demo')
from pr2_simple_interface import *
from sound_play.msg import SoundRequest
from sound_play.libsoundplay import SoundClient
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
sound = SoundClient()

rospy.sleep(1.)

head.look_at(1.0, 0.0, 1.0)
head.wait_for()

while True:
    head.random_look_at_face()
    head.wait_for()
    rospy.sleep(3.)


