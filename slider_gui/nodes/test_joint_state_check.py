#!/usr/bin/env python

import roslib; 
#roslib.load_manifest('planning_environment')
roslib.load_manifest('slider_gui')
import rospy
import arm_navigation_msgs.srv
from arm_navigation_msgs.srv import GetStateValidity,GetStateValidityRequest,SetPlanningSceneDiff,SetPlanningSceneDiffRequest
from pr2_controllers_msgs.msg import JointTrajectoryControllerState
from pr2_controllers_msgs.msg import JointControllerState
from sensor_msgs.msg import JointState 

def currentJointState():
    '''
    Utility to find current state of all joints.
    Returns /joint_states message instance
    '''
    try:
        # Args: Topic, topic_type, timeout=None:
        
        jointState = rospy.wait_for_message("/joint_states",
                                            JointState, 
                                            5); #sec
    except rospy.ROSException:
        # Timeout:
        return None;
    except rospy.ROSInterruptException:
        # Shutdown interrupt waiting:
        return None;
    return jointState
  
  
# ------------------------------------------- Initialization ---------------------------------  

rospy.init_node('test_collision_check')

# One-time initialization of the environment server:

SET_PLANNING_SCENE_DIFF_NAME = "/environment_server/set_planning_scene_diff"
rospy.wait_for_service(SET_PLANNING_SCENE_DIFF_NAME);
set_planning_scene_server = rospy.ServiceProxy(SET_PLANNING_SCENE_DIFF_NAME, SetPlanningSceneDiff)
# Just send an empty message: 
res = set_planning_scene_server.call(SetPlanningSceneDiffRequest())

# One-time initialization of the state validity server: 
rospy.wait_for_service('/planning_scene_validity_server/get_state_validity');
state_validity_server = rospy.ServiceProxy('/planning_scene_validity_server/get_state_validity', GetStateValidity)

# Set up a request message for state checks:
state_req = GetStateValidityRequest()
state_req.check_collisions = True
state_req.robot_state.joint_state.header.stamp = rospy.Time.now()

# -------------------------------------------- Usage ----------------------------------------

# Get the current joint state, which is presumably collision free:
currState = currentJointState();
print str(currState)

# Set the joint name and joint value arrays in the state check request message
# just for the right shoulder joints. The joint service uses the current joint
# states for any unfilled joint fields in the request message:

for name in currState.name:
    state_req.robot_state.joint_state.name.append(name)
    state_req.robot_state.joint_state.position.append(currState.position[currState.name.index(name)])

#state_req.robot_state.joint_state.name = ['r_shoulder_pan_joint', 'r_shoulder_lift_joint']
#state_req.robot_state.joint_state.position = [currState.position[currState.name.index('r_shoulder_pan_joint')],
#                                              currState.position[currState.name.index('r_shoulder_lift_joint')]]
# Ask for the joint state check:
print str(state_req)
res = state_validity_server.call(state_req)
# Assuming the current state of the right shoulder is OK,
# we should get an OK back:
if res.error_code.val != res.error_code.SUCCESS:
    print "False positive: Incorrectly reports collision when collision-free: " + str(res.error_code.val)
else:
    print "Correctly reports collision-free scenario."

# Check state when all right-arm joints are at zero:
state_req.robot_state.joint_state.name = ['r_shoulder_pan_joint', 'l_shoulder_pan_joint']

state_req.robot_state.joint_state.position = [-1.0, 1.0]
#print str(state_req)

res = state_validity_server.call(state_req)
if (res.error_code.val == res.error_code.SUCCESS):
  print "False negative: Reports collision-free with collision: " + str(res.error_code.val) 
elif (res.error_code.val < 1):
  print "Properly reports collision."

print "Done testing joint state check."



