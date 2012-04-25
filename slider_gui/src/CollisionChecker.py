import roslib
roslib.load_manifest('slider_gui')
import rospy

from arm_navigation_msgs.srv import GetStateValidity, GetStateValidityRequest, SetPlanningSceneDiff, SetPlanningSceneDiffRequest
from pr2_controllers_msgs.msg import JointControllerState, JointTrajectoryControllerState
from sensor_msgs.msg import JointState 

class CollisionChecker:

    def __init__(self):
        # initialization of the environment server
        SET_PLANNING_SCENE_DIFF_NAME = '/environment_server/set_planning_scene_diff'
        rospy.wait_for_service(SET_PLANNING_SCENE_DIFF_NAME)
        set_planning_scene_server = rospy.ServiceProxy(SET_PLANNING_SCENE_DIFF_NAME, SetPlanningSceneDiff)
        # just send an empty message
        res = set_planning_scene_server.call(SetPlanningSceneDiffRequest())

        # initialization of the state validity server
        rospy.wait_for_service('/planning_scene_validity_server/get_state_validity')
        self._state_validity_server = rospy.ServiceProxy('/planning_scene_validity_server/get_state_validity', GetStateValidity)

    def is_current_state_in_collision(self):
        state_req = self._create_state_validity_request()

        # ask for the joint state check
        res = self._state_validity_server.call(state_req)
        return res.error_code.val != res.error_code.SUCCESS

    def is_in_collision(self, joint_values):
        state_req = self._create_state_validity_request()

        # set the joint name and joint value arrays in the state check request message
        for name, position in joint_values.items():
            if name in state_req.robot_state.joint_state.name:
                index = state_req.robot_state.joint_state.name.index(name)
                state_req.robot_state.joint_state.position[index] = position
            else:
                state_req.robot_state.joint_state.name.append(name)
                state_req.robot_state.joint_state.position.append(position)
        #print str(state_req.robot_state.joint_state.name)
        #print str(state_req.robot_state.joint_state.position)

        # ask for the joint state check
        res = self._state_validity_server.call(state_req)
        return res.error_code.val != res.error_code.SUCCESS

    def _create_state_validity_request(self):
        # set up a request message for state checks
        state_req = GetStateValidityRequest()
        state_req.check_collisions = True
        state_req.robot_state.joint_state.header.stamp = rospy.Time.now()

        # get the current joint state
        currState = self._current_joint_state()
        #print str(currState)
        if currState is None:
            return None

        # set the joint name and joint value arrays in the state check request message
        for name in currState.name:
            state_req.robot_state.joint_state.name.append(name)
            state_req.robot_state.joint_state.position.append(currState.position[currState.name.index(name)])

        #print str(state_req)
        return state_req

    def _current_joint_state(self):
        '''
        Utility to find current state of all joints.
        Returns /joint_states message instance
        '''
        try:
            joint_state = rospy.wait_for_message('/joint_states', JointState, 3)
        except rospy.ROSException:
            # timeout
            return None
        except rospy.ROSInterruptException:
            # shutdown interrupt waiting
            return None
        return joint_state
