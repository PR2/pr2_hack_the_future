#!/usr/bin/env python
# Scroll down for main program

import roslib
roslib.load_manifest('pr2_simple_interface')
#import rospy

#rospy.init_node('steve_demo')
from pr2_simple_interface import *

start()
#############################################################
#############################################################
#############################################################
#############################################################
#############################################################
#############################################################
#############################################################
#############################################################
#############################################################
#############################################################
#############################################################
#############################################################
#############################################################
#############################################################
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


#arm.move_to([-1.42, 0.640, 0.647, -1.925, 30.931, -0.521, -16.642], RIGHT)
#arm.move_to([1.54, 0.028, -0.061, -1.932, -0.969, -0.226, -3.541], LEFT)
# converted to degrees
arm.move_to([-81.360006908576892, 36.669298888372687, 37.070369344964263, -110.29437556268347, 1772.2157561191493, -29.851101126315889, -953.51636265671596], RIGHT)
arm.move_to([88.235500450146773, 1.604281826366305, -3.495042550298022, -110.69544601927504, -55.519610348176769, -12.948846169956605, -202.88435525582452], LEFT)
arm.wait_for(BOTH)

gripper.rel(BOTH)
gripper.wait_for(BOTH)

rospy.sleep(2.0)

#arm.move_to([-0.439, -0.268, -0.060, -1.059, 30.217, -0.098, -17.139], RIGHT)
#arm.move_to([0.347, -0.334, 0.214, -0.871, -0.553, -0.511, -3.173], LEFT)
# converted to degrees
arm.move_to([-25.152847206243138, -15.355268909506064, -3.4377467707849392, -60.676230504354173, 1731.3065695468083, -5.6149863922820682, -981.9923650747179], RIGHT)
arm.move_to([19.881635491039564, -19.136790357369499, 12.261296815799616, -49.904623955894706, -31.684566070734526, -29.278143331185067, -181.79950839501021], LEFT)


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

#arm.move_to([1.458, 1.044, 0.081, -2.111, -1.339, -0.168, -3.500], LEFT)
#arm.move_to([-1.227, 0.989, 0.279, -1.985, 31.095, -0.754, -17.206], RIGHT)
# converted to degrees
arm.move_to([83.537246530074029, 59.816793811657952, 4.6409581405596683, -120.95139055211679, -76.719048768017231, -9.6256909581978309, -200.53522829578813], LEFT)
arm.move_to([-70.301921462552016, 56.665525938438421, 15.985522484149969, -113.73212233346841, 1781.6122639592948, -43.201017752864068, -985.83118230209448], RIGHT)
arm.wait_for(BOTH)

gripper.close(BOTH)
gripper.wait_for(BOTH)

head.look_at(1.0, 0.0, 1.0)
head.wait_for()

