
# Pose 0:
head.look(0,40,2.80)
torso.set(0.12, 2.80)
arm.move_to([55,25,64,-63,-11,-67,0],LEFT,2.80)
gripper.pose(LEFT,3.9)
arm.move_to([-55,25,-64,-63,11,-67,0],RIGHT,2.80)
gripper.pose(RIGHT,3.9)
arm.wait_for(BOTH)
gripper.wait_for(BOTH)

# Pose 1:
head.look(0,22,2.80)
torso.set(0.12, 2.80)
arm.move_to([44,22,179,-73,-43,-67,0],LEFT,2.80)
gripper.pose(LEFT,3.9)
arm.move_to([-44,22,-179,-73,43,-67,0],RIGHT,2.80)
gripper.pose(RIGHT,3.9)
arm.wait_for(BOTH)
gripper.wait_for(BOTH)

# Pose 2:
head.look(0,14,2.80)
torso.set(0.12, 2.80)
arm.move_to([58,3,109,-94,9,-67,0],LEFT,2.80)
gripper.pose(LEFT,3.9)
arm.move_to([-44,22,-179,-73,43,-67,0],RIGHT,2.80)
gripper.pose(RIGHT,3.9)
arm.wait_for(BOTH)
gripper.wait_for(BOTH)

# Pose 3:
head.look_at_face()

# Pose 4:
# Puppet function playMusic could not be translated to Python.

# Pose 5:
# Puppet function playSound could not be translated to Python.

# Pose 6:
sound.say('Hello World')

