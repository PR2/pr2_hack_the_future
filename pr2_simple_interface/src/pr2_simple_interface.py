#!/usr/bin/env python
# Scroll down for main program

import roslib
roslib.load_manifest('pr2_simple_interface')
import rospy
import actionlib
import math
import random

from pr2_controllers_msgs.msg import *
from pr2_gripper_sensor_msgs.msg import *
from pr2_controllers_msgs.msg import *
from trajectory_msgs.msg import *
from sensor_msgs.msg import *
from sound_play.msg import SoundRequest
from sound_play.libsoundplay import SoundClient
from face_detector.msg import *
import std_srvs.srv


LEFT = 1
RIGHT = 2
BOTH = 3

def convert_s(s):
    if (s == LEFT):
        return "left"
    if (s == RIGHT):
        return "right"
    if (s == BOTH):
        return "both"
    return "ERROR"

#def Face():
#    c = actionLib.SimpleActionClient('face_detector_action',face_detector.msg.FaceDetectorAction)
#    c.wait_for_server()
#    return c

def TrajClient(t):
    print t
    c = actionlib.SimpleActionClient(t, JointTrajectoryAction)
    c.wait_for_server()
    return c

def PlaceClient(t):
    print t
    c = actionlib.SimpleActionClient(t, PR2GripperEventDetectorAction)
    c.wait_for_server()
    return c

def GripperClient(t):
    print t
    c = actionlib.SimpleActionClient(t, Pr2GripperCommandAction)
    c.wait_for_server()
    return c

class Gripper:
    def __init__(self):
        pass
    
    def rel(self, s):
        print "Release gripper:", convert_s(s)
        openg = Pr2GripperCommandGoal()
        openg.command.position = 0.09    # position open (9 cm)
        openg.command.max_effort = -1.0  # Do not limit effort (negative)
        if(s == LEFT or s == BOTH):
            gripper_client_l.send_goal(openg)
        if(s == RIGHT or s == BOTH):
            gripper_client_r.send_goal(openg)

    #Close the gripper
    def close(self, s):
        print "Close gripper:", convert_s(s) 
        close = Pr2GripperCommandGoal()
        close.command.position = 0.002    # position open (9 cm)
        close.command.max_effort = -1.0  # Do not limit effort (negative)
        if(s == LEFT or s == BOTH):
            gripper_client_l.send_goal(close)
        if(s == RIGHT or s == BOTH):
            gripper_client_r.send_goal(close)

    def wait_for(self, s):
        if(s == LEFT or s == BOTH):
            gripper_client_l.wait_for_result()
        if(s == RIGHT or s == BOTH):
            gripper_client_r.wait_for_result()

    # //move into place mode to drop an object
    def slap(self, s):
        place_goal = PR2GripperEventDetectorGoal()
        place_goal.command.trigger_conditions = 4 # use just acceleration as our contact signal
        place_goal.command.acceleration_trigger_magnitude = 6.0 # m/^2
        place_goal.command.slip_trigger_magnitude = 0.008 # slip gain
        
        if sim:
            rospy.sleep(1)
            print "Simulating a slap"
            self.rel(s)
        else:
            if(s == LEFT or s == BOTH):
                place_client_l.send_goal(place_goal)
            if(s == RIGHT or s == BOTH):
                place_client_r.send_goal(place_goal)

    #Returns the current state of the action
    def slapDone(self, s):
        if sim:
           print "slapDone always true in simulation"
           return True
        else:
           SUCCEEDED = 3 #Hack
           if(s == BOTH):
               return place_client_l.get_state() == SUCCEEDED and place_client_r.get_state() == SUCCEEDED
           elif(s == LEFT):
               return place_client_l.get_state() == SUCCEEDED
           elif(s == RIGHT):
               return place_client_r.get_state() == SUCCEEDED
           return False

    def wait_for_slap(self, s):
        print "Wait for slap: ", convert_s(s)
        self.slap(s)
        while not self.slapDone(s):
            rospy.sleep(0.01)

    def determine_slap(self):
        print "Determine which arm is slapped"
        self.slap(BOTH)
        while True:
            if (self.slapDone(LEFT)):
                print "Left arm slapped"
                return LEFT
            if (self.slapDone(RIGHT)):
                print "Right arm slapped"
                return RIGHT
            rospy.sleep(0.01)


class RobotArm:
    def __init__(self):
        pass
    
    def startTrajectory(self, goal, right_arm):
        #//Start the trjaectory immediately
        goal.trajectory.header.stamp = rospy.get_rostime()
        if(right_arm):
            traj_client_r.send_goal(goal)
        else:
            traj_client_l.send_goal(goal)

    def arm_trajectoryPoint(self, angles, duration, right_arm):
        goal = JointTrajectoryGoal();
        # First, the joint names, which apply to all waypoints
        #starts at 17
        if(right_arm):
            goal.trajectory.joint_names = ["r_shoulder_pan_joint", "r_shoulder_lift_joint", "r_upper_arm_roll_joint", "r_elbow_flex_joint", "r_forearm_roll_joint", "r_wrist_flex_joint", "r_wrist_roll_joint"]
        else:
            goal.trajectory.joint_names = ["l_shoulder_pan_joint", "l_shoulder_lift_joint", "l_upper_arm_roll_joint", "l_elbow_flex_joint", "l_forearm_roll_joint", "l_wrist_flex_joint", "l_wrist_roll_joint"]
            
        # We will have N waypoints in this goal trajectory
        goal.trajectory.points = [JointTrajectoryPoint() ];

        # First trajectory point
        # Positions
        ind = 0
        goal.trajectory.points[ind].positions = [ a * math.pi / 180.0 for a in angles ]
        # Velocities
        goal.trajectory.points[ind].velocities = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        goal.trajectory.points[ind].time_from_start = rospy.Duration.from_sec(duration)
        #we are done; return the goal
        return goal

    def move_to(self, goal, s):
        arm = False
        if (s == RIGHT):
            print "Moving right arm to:", goal
            arm = True
        if (s == LEFT):
            print "Moving left arm to:", goal
        if (s == BOTH):
            print "WARNING: you can't send a goal of both to the arms"
        self.startTrajectory(self.arm_trajectoryPoint(goal, 2.0, arm), arm)

    def wait_for(self, s):
        print "Wait for arm:", convert_s(s)
        left = False
        right = False
        if (s == LEFT or s == BOTH):
            left = True
        if (s == RIGHT or s == BOTH):
            right = True
        if(left):
            traj_client_l.wait_for_result()
        if(right):
            traj_client_r.wait_for_result()


class Head:
    def __init__(self):
        pass

    def look_at(self, x, y, z):
        print "Look at:", x, y, z
        g = PointHeadGoal()
        g.target.header.frame_id = 'base_link'
        g.target.point.x = x
        g.target.point.y = y
        g.target.point.z = z
        g.min_duration = rospy.Duration(1.0)
        head_client.send_goal(g)

    def look_at_face(self):
        print "Looking at a face"
        fgoal = FaceDetectorGoal()
        nfaces = 0            
        closest = -1
        closest_dist = 1000
        while nfaces < 1:
            face_client.send_goal(fgoal)
            face_client.wait_for_result()
            f = face_client.get_result()
            nfaces = len(f.face_positions)
                   
            for i in range(nfaces):
                dist = f.face_positions[i].pos.x*f.face_positions[i].pos.x + f.face_positions[i].pos.y*f.face_positions[i].pos.y\
 + f.face_positions[i].pos.z*f.face_positions[i].pos.z
                if dist < closest_dist:
                    closest = i
                    closest_dist = dist
            
            if closest > -1:
                g = PointHeadGoal()
                g.target.header.frame_id = f.face_positions[closest].header.frame_id
                g.target.point.x = f.face_positions[closest].pos.x
                g.target.point.y = f.face_positions[closest].pos.y
                g.target.point.z = f.face_positions[closest].pos.z
                g.min_duration = rospy.Duration(1.0)
                head_client.send_goal(g)


    def random_look_at_face(self):
        print "Looking at a face"
        fgoal = FaceDetectorGoal()
        nfaces = 0
        closest = -1
        closest_dist = 1000
        while nfaces < 1:
            face_client.send_goal(fgoal)
            face_client.wait_for_result()
            f = face_client.get_result()
            nfaces = len(f.face_positions)

            iter = 0
            while iter < 10 and closest <= -1:
                iter = iter + 1
                i = random.randrange(0,nfaces)

                dist = f.face_positions[i].pos.x*f.face_positions[i].pos.x + f.face_positions[i].pos.y*f.face_positions[i].pos.y\
 + f.face_positions[i].pos.z*f.face_positions[i].pos.z
                print "This face has position and dist ", f.face_positions[i].pos.x, f.face_positions[i].pos.y, f.face_positions[i].pos.z, dist
                if dist < closest_dist and f.face_positions[i].pos.y > -1.0 :
                    closest = i
                    closest_dist = dist
                    break

            if closest > -1:
                print "Turning to face ",  f.face_positions[closest].pos.x,  f.face_positions[closest].pos.y,  f.face_positions[closest].pos.z
                g = PointHeadGoal()
                g.target.header.frame_id = f.face_positions[closest].header.frame_id
                g.target.point.x = f.face_positions[closest].pos.x
                g.target.point.y = f.face_positions[closest].pos.y
                g.target.point.z = f.face_positions[closest].pos.z
                g.min_duration = rospy.Duration(1.0)
                head_client.send_goal(g)


    def wait_for(self):
        print "Wait for head positioning"
        head_client.wait_for_result()

class Torso:
    def __init__(self):
        self.torso_pub = rospy.Publisher('torso_controller/command', JointTrajectory)
        self.torso_sub = rospy.Subscriber('joint_states', JointState, self.joint_state_callback)
        self.pos = -1.0

    def joint_state_callback(self, msg):
        for i in range(0, len(msg.name)):
            if (msg.name[i] == "torso_lift_joint"):
                self.pos = msg.position[i]

    def set(self, h):
        print "Setting torso height to", h
        while not rospy.is_shutdown() and (self.pos > h + 0.02 or self.pos < h - 0.02):
            traj = JointTrajectory()
            traj.header.stamp = rospy.get_rostime()
            traj.joint_names.append("torso_lift_joint");
            traj.points.append(JointTrajectoryPoint())
            traj.points[0].positions.append(h)
            traj.points[0].velocities.append(0.1)
            traj.points[0].time_from_start = rospy.Duration(0.2)
            self.torso_pub.publish(traj)
            rospy.sleep(rospy.Duration.from_sec(0.2))

def hug():
   rospy.wait_for_service('/pr2_props/hug')
   hug_client = rospy.ServiceProxy('/pr2_props/hug', std_srvs.srv.Empty)
   try:
      hug_client()
      print "Hug successful!"
   except rospy.ServiceException, e:
      print "Hug service call failed"


def start():
  print "Initializing pr2_simple_interface"
  rospy.init_node('pr2_simple_interface')
  
  print "Sound Client"
  global sound
  sound = SoundClient()
  
  global traj_client_r
  global traj_client_l
  global place_client_r
  global place_client_l
  traj_client_r = TrajClient("r_arm_controller/joint_trajectory_action")
  traj_client_l = TrajClient("l_arm_controller/joint_trajectory_action")

  global gripper_client_l
  global gripper_client_r
  global sim
  # only user gripper sensors if we aren't in simulation
  if( not rospy.has_param('gazebo') ):
    gripper_client_l = GripperClient("l_gripper_sensor_controller/gripper_action")
    gripper_client_r = GripperClient("r_gripper_sensor_controller/gripper_action")
    place_client_l = PlaceClient("l_gripper_sensor_controller/event_detector")
    place_client_r = PlaceClient("r_gripper_sensor_controller/event_detector")
    sim = False
  else:
    gripper_client_l = GripperClient("l_gripper_controller/gripper_action")
    gripper_client_r = GripperClient("r_gripper_controller/gripper_action")
    sim = True

  global face_client
  face_client = actionlib.SimpleActionClient('face_detector_action',FaceDetectorAction)
  face_client.wait_for_server()
  print "Got face server"

  global head_client
  head_client = actionlib.SimpleActionClient('/head_traj_controller/point_head_action', PointHeadAction)
  head_client.wait_for_server()
  
  print "pr2_simple_interface init done"


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
# head.look_at_face()
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
