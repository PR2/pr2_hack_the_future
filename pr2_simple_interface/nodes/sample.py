#!/usr/bin/env python

# startup code
import roslib
roslib.load_manifest('pr2_simple_interface')
from pr2_simple_interface import *
start()

# create robot objects
gripper = Gripper()
arm = RobotArm()
head = Head()
torso = Torso()
sound = Sound()

# robot speaks
sound.say("I'm Kiko.")

# nod head
for i in range(2):
   head.look_at(1.0, 0.0, 0.5) # look down
   head.wait_for()             # wait for head to stop moving
   head.look_at(1.0, 0.0, 1.5) # look up
   head.wait_for()

# look straight ahead
head.look_at(1.0, 0.0, 1.0)
head.wait_for()

# move arms individually
arm.move_to([-80, 40, 30, -110, 0, 0, -5], RIGHT)
arm.mirror(RIGHT)
arm.wait_for(BOTH)

# open grippers
gripper.release(BOTH)
gripper.wait_for(BOTH)

# mirror arms
arm.move_to([-20, -15, -20, -50, 30, -30, -180], RIGHT)
arm.move_to([20, -15, 20, -50, -30, -30, 180], LEFT)

# start talking before arms are done moving
sound.say("Double high fives.")

# wait for arms to finish moving
arm.wait_for(BOTH)

# wait for a slap on both hands
gripper.wait_for_slap(BOTH)

# nod up
head.look_at(1.0, 0.0, 1.5)
head.wait_for()
head.look_at(1.0, 0.0, 1.0)
head.wait_for()

sound.say("Slap one hand.")

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

sound.say("Goodbye.")
