#!/usr/bin/env python

import roslib; roslib.load_manifest('pr2_joint_teleop')
import rospy
import actionlib
import yaml
import math

from pr2_controllers_msgs.msg import JointTrajectoryGoal, JointTrajectoryAction
from trajectory_msgs.msg import JointTrajectoryPoint
from pr2_controllers_msgs.msg import Pr2GripperCommand
from sensor_msgs.msg import Joy

def pose_(position, joints, client):
   goal = JointTrajectoryGoal()
   goal.trajectory.joint_names = joints

   goal.trajectory.points = [ JointTrajectoryPoint() ]

   goal.trajectory.points[0].velocities = [0.0] * len(joints);
   goal.trajectory.points[0].positions = [0.0] * len(joints);
   for i, j in enumerate(joints):
      goal.trajectory.points[0].positions[i] = position[i]

   goal.trajectory.points[0].time_from_start = rospy.Duration.from_sec(1.0)
   client.send_goal(goal)

def scale(axes, min_limit, max_limit):
   position = [ 0 ] * len(min_limit)
   for i in range(len(min_limit)):
      avg = (min_limit[i] + max_limit[i]) / 2.0
      r = (max_limit[i] - min_limit[i]) / 2.0
      deg = axes[i]*r + avg
      position[i] = deg * math.pi / 180.0
   return position

def pose_r(axes):
   joints = ["r_shoulder_pan_joint", "r_shoulder_lift_joint", "r_upper_arm_roll_joint", "r_elbow_flex_joint", "r_forearm_roll_joint", "r_wrist_flex_joint", "r_wrist_roll_joint"]
   min_limit = [ -130, -30, -224, -130, -180, -130, -180 ]
   max_limit = [ 0, 80, 44, 0, 180, 0, 180 ]
   position = scale(axes, min_limit, max_limit)
   pose_(position, joints, traj_client_r)

def pose_l(axes):
   joints = ["l_shoulder_pan_joint", "l_shoulder_lift_joint", "l_upper_arm_roll_joint", "l_elbow_flex_joint", "l_forearm_roll_joint", "l_wrist_flex_joint", "l_wrist_roll_joint"]
   min_limit = [0, -30, -44, -130, -180, -130, -180 ]
   max_limit = [130, 80, 224, 0, 180, 0, 180 ]
   position = scale(axes, min_limit, max_limit)
   pose_(position, joints, traj_client_l)

def pose_head(axes):
   joints = [ 'head_pan_joint', 'head_tilt_joint' ]
   min_limit = [ -160, -30 ]
   max_limit = [ 160, 90 ]
   position = scale(axes, min_limit, max_limit)
   pose_(position, joints, traj_client_head)

def pose_torso(axes):
   joints = [ 'torso_lift_joint' ]
   position = [ (axes[0] + 1.0) * 0.5 * 0.3 ]
   pose_(position, joints, traj_client_torso)

def pose_gripper_l(axes):
   control = Pr2GripperCommand(0.04, 0)
   control.max_effort = 10.0
   control.position = 0.08 * ((axes[0] + 1.0) / 2.0)
   gripper_l.publish(control)

def pose_gripper_r(axes):
   control = Pr2GripperCommand(0.04, 0)
   control.max_effort = 10.0
   control.position = 0.08 * ((axes[0] + 1.0) / 2.0)
   gripper_r.publish(control)

# move the robot to the specified pose
def pose(position):
   pose_l(position)
   pose_r(position)
   pose_head(position)
   pose_torso(position)

def TrajClient(t):
   c = actionlib.SimpleActionClient(t, JointTrajectoryAction)
   c.wait_for_server()
   return c

def joy_callback(joy_msg):
   if len(joy_msg.buttons) >= 25:
      mode = joy_msg.buttons[24]
      if mode == 0:
         right = []
         right.extend(joy_msg.axes[2:8])
         right.append(joy_msg.axes[17])
         right.append(joy_msg.axes[8])
         right[1] = -right[1]
         left = []
         left.extend(joy_msg.axes[2:8])
         left.append(joy_msg.axes[17])
         left.append(joy_msg.axes[8])
         left[0] = -left[0]
         left[1] = -left[1]
         left[2] = -left[2]
         left[4] = -left[4]
         left[6] = -left[6]
   
         pose_r(right[0:7])
         pose_l(left[0:7])
         pose_gripper_r(right[7:8])
         pose_gripper_l(left[7:8])
         pose_head([-joy_msg.axes[9], -joy_msg.axes[0]])
         pose_torso([joy_msg.axes[1]])
      elif mode == 1:
         a = joy_msg.axes
         right = [ a[17], a[8], a[16], a[7], a[15], a[6], a[14], a[5] ]
         right[1] = -right[1]
         left = [ a[9], a[0], a[10], a[1], a[11], a[2], a[12], a[3] ]
         left[0] = -left[0]
         left[1] = -left[1]
         left[2] = -left[2]
         left[4] = -left[4]
         left[6] = -left[6]
   
         pose_r(right[0:7])
         pose_l(left[0:7])
         pose_gripper_r(right[7:8])
         pose_gripper_l(left[7:8])
         pose_head([ -a[13], -a[4] ])
         #pose_torso([joy_msg.axes[1]])

if __name__ == '__main__':
   rospy.init_node('pr2_joint_teleop')
   argv = rospy.myargv()

   traj_client_r = TrajClient("r_arm_controller/joint_trajectory_action")
   traj_client_l = TrajClient("l_arm_controller/joint_trajectory_action")
   traj_client_head = TrajClient('head_traj_controller/joint_trajectory_action')
   traj_client_torso = TrajClient('torso_controller/joint_trajectory_action')
   gripper_l = rospy.Publisher("l_gripper_controller/command", Pr2GripperCommand)
   gripper_r = rospy.Publisher("r_gripper_controller/command", Pr2GripperCommand)

   rospy.Subscriber("joy", Joy, joy_callback)
   rospy.spin()
