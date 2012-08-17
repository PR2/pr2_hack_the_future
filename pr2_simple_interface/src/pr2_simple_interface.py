#!/usr/bin/env python
# Scroll down for main program

import roslib
roslib.load_manifest('pr2_simple_interface')
import rospy
import actionlib
import math

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

def pose_(position, joints, client, dur):
   goal = JointTrajectoryGoal()
   goal.trajectory.joint_names = joints

   goal.trajectory.points = [ JointTrajectoryPoint() ]

   goal.trajectory.points[0].velocities = [0.0] * len(joints);
   goal.trajectory.points[0].positions = position;
   goal.trajectory.points[0].time_from_start = rospy.Duration.from_sec(dur)
   client.send_goal(goal)

def pose_r(position, dur):
   joints = ["r_shoulder_pan_joint", "r_shoulder_lift_joint", "r_upper_arm_roll_joint", "r_elbow_flex_joint", "r_forearm_roll_joint", "r_wrist_flex_joint", "r_wrist_roll_joint"]
   pose_(position, joints, traj_client_r, dur)

def pose_l(position, dur):
   joints = ["l_shoulder_pan_joint", "l_shoulder_lift_joint", "l_upper_arm_roll_joint", "l_elbow_flex_joint", "l_forearm_roll_joint", "l_wrist_flex_joint", "l_wrist_roll_joint"]
   pose_(position, joints, traj_client_l, dur)

def pose_head(position, dur):
   joints = [ 'head_pan_joint', 'head_tilt_joint' ]
   pose_(position, joints, traj_client_head, dur)

def pose_torso(position, dur):
   joints = [ 'torso_lift_joint' ]
   pose_(position, joints, traj_client_torso, dur)

def actionClient(topic, t):
    if debug:
      print t
    c = actionlib.SimpleActionClient(topic, t)
    c.wait_for_server()
    return c

def TrajClient(t):
    return actionClient(t, JointTrajectoryAction)

def GripperClient(t):
    return actionClient(t, Pr2GripperCommandAction)

class Gripper:
    def __init__(self):
        pass
    
    # release the gripper
    def rel(self, s):
        print "Please use gripper.release() instead of gripper.rel()"
        self.release(s)

    # release the gripper
    def release(self, s):
        print "Release gripper:", convert_s(s)
        self.pose(s, 0.09) # position open (9 cm)

    # open the gripper to a specified position
    def pose(self, s, pos):
        if pos < 0:
           pos = 0
        if pos > 0.09:
           pos = 0.09

        print "Set gripper %s to %d"%(convert_s(s), pos)
        openg = Pr2GripperCommandGoal()
        openg.command.position = pos    # position
        openg.command.max_effort = -1.0  # Do not limit effort (negative)
        if(s == LEFT or s == BOTH):
            gripper_client_l.send_goal(openg)
        if(s == RIGHT or s == BOTH):
            gripper_client_r.send_goal(openg)

    # close the gripper
    def close(self, s):
        print "Close gripper:", convert_s(s) 
        self.pose(s, 0.002) # closed position (0.002m spacing)

    # wait for the gripper action to complete
    def wait_for(self, s):
        if(s == LEFT or s == BOTH):
            gripper_client_l.wait_for_result()
        if(s == RIGHT or s == BOTH):
            gripper_client_r.wait_for_result()

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

    def move_to(self, goal, s, dur=2.0):
        positions = [ a * math.pi / 180.0 for a in goal ]
        if (s == RIGHT):
            print "Moving right arm to:", goal
            pose_r(positions, dur)
            arm = True
        if (s == LEFT):
            print "Moving left arm to:", goal
            pose_l(positions, dur)
        if (s == BOTH):
            print "WARNING: you can't send a goal of both to the arms"

    def wait_for(self, s):
        print "Wait for arm:", convert_s(s)
        if (s == LEFT or s == BOTH):
            traj_client_l.wait_for_result()
        if (s == RIGHT or s == BOTH):
            traj_client_r.wait_for_result()


class Head:
    def __init__(self):
        self.mode = 0
        pass

    def look_at(self, x, y, z, dur=1.0):
        print "Look at:", x, y, z
        g = PointHeadGoal()
        g.target.header.frame_id = 'base_link'
        g.target.point.x = x
        g.target.point.y = y
        g.target.point.z = z
        g.min_duration = rospy.Duration(dur)
        head_client.send_goal(g)
        self.mode = 1

    def look(self, x, y, dur=1.0):
        pose = [ x * math.pi / 180.0, y * math.pi / 180.0 ]
        print "Look: ", pose
        pose_head(pose, dur)
        self.mode = 2

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
                self.mode = 1
        
    def wait_for(self):
        print "Wait for head positioning"
        if self.mode == 1:
            head_client.wait_for_result()
        if self.mode == 2:
            traj_client_head.wait_for_result()
        self.mode = 0
            

class Torso:
    def __init__(self):
       pass

    def set(self, h, dur=10.0):
        if h > 0.3:
           h = 0.3
        if h < 0:
           h = 0
        print "Setting torso height to", h
        pose_torso([h], dur)

    def wait_for(self):
       print "Waiting for torso"
       torso_client.wait_for_result();

class Sound(SoundClient):
   def __init__(self):
      if debug:
         print "Initializing Sound Client"
      SoundClient.__init__(self)
      # wait for subscribers
      timeout = 10
      while timeout > 0:
         if self.pub.get_num_connections() > 0:
            timeout = 0
         else:
            rospy.sleep(1)

   def say(self, text):
      print "Saying: \"%s\""%text
      SoundClient.say(self, text)

def hug():
   rospy.wait_for_service('/pr2_props/hug')
   hug_client = rospy.ServiceProxy('/pr2_props/hug', std_srvs.srv.Empty)
   try:
      hug_client()
      print "Hug successful!"
   except rospy.ServiceException, e:
      print "Hug service call failed"


def start(d = False):
  global debug
  debug = d
  if debug:
      print "Initializing pr2_simple_interface"
  rospy.init_node('pr2_simple_interface')

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
    place_client_l = actionClient("l_gripper_sensor_controller/event_detector", PR2GripperEventDetectorAction)
    place_client_r = actionClient("r_gripper_sensor_controller/event_detector", PR2GripperEventDetectorAction)
    sim = False
  else:
    gripper_client_l = GripperClient("l_gripper_controller/gripper_action")
    gripper_client_r = GripperClient("r_gripper_controller/gripper_action")
    sim = True

  global face_client
  face_client = actionClient('face_detector_action',FaceDetectorAction)

  global head_client
  head_client = actionClient('/head_traj_controller/point_head_action', PointHeadAction)

  global traj_client_torso
  traj_client_torso = TrajClient('torso_controller/joint_trajectory_action')

  global traj_client_head
  traj_client_head = TrajClient('head_traj_controller/joint_trajectory_action')

  
  if debug:
     print "pr2_simple_interface init done"
