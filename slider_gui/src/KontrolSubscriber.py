from actions.ActionSet import ActionSet
from actions.Pr2MoveHeadAction import Pr2MoveHeadAction
from actions.Pr2MoveLeftArmAction import Pr2MoveLeftArmAction
from actions.Pr2MoveLeftGripperAction import Pr2MoveLeftGripperAction
from actions.Pr2MoveRightArmAction import Pr2MoveRightArmAction
from actions.Pr2MoveRightGripperAction import Pr2MoveRightGripperAction
from actions.Pr2MoveTorsoAction import Pr2MoveTorsoAction
import rospy
from sensor_msgs.msg import Joy
from Signal import Signal

class KontrolSubscriber(object):

    def __init__(self):
        self._current_msg = None
        rospy.Subscriber('joy', Joy, self._joy_callback)
        self.new_message = Signal()

    def _joy_callback(self, joy_msg):
        self._current_msg = joy_msg
        self.new_message.emit()

    def get_action_set(self):
        set = ActionSet()
        if len(self._current_msg.buttons) >= 25:
            mode = self._current_msg.buttons[24]
            if mode == 0:
                head = Pr2MoveHeadAction()
                head_data = [-self._current_msg.axes[9], -self._current_msg.axes[0]]
                self._set_transformed_data(head, head_data)
                set.add_action(head)

                torso = Pr2MoveTorsoAction()
                torso_data = [self._current_msg.axes[1]]
                self._set_transformed_data(torso, torso_data)
                set.add_action(torso)

                lgrip = Pr2MoveLeftGripperAction()
                lgrip_data = [self._current_msg.axes[8]]
                self._set_transformed_data(lgrip, lgrip_data)
                set.add_action(lgrip)

                rgrip = Pr2MoveRightGripperAction()
                rgrip_data = lgrip_data
                self._set_transformed_data(rgrip, rgrip_data)
                set.add_action(rgrip)

                rarm_data = []
                rarm_data.extend(self._current_msg.axes[2:8])
                rarm_data.append(self._current_msg.axes[17])
                rarm_data[1] = -rarm_data[1]

                rarm = Pr2MoveRightArmAction()
                self._set_transformed_data(rarm, rarm_data)
                set.add_action(rarm)

                larm_data = []
                larm_data.extend(self._current_msg.axes[2:8])
                larm_data.append(self._current_msg.axes[17])
                larm_data[0] = -larm_data[0]
                larm_data[1] = -larm_data[1]
                larm_data[2] = -larm_data[2]
                larm_data[4] = -larm_data[4]
                larm_data[6] = -larm_data[6]

                larm = Pr2MoveLeftArmAction()
                self._set_transformed_data(larm, larm_data)
                set.add_action(larm)


#
#                pose_r(right[0:7])
#                pose_l(left[0:7])
#                pose_gripper_r(right[7:8])
#                pose_gripper_l(left[7:8])
        return set

    def _set_transformed_data(self, action, data):
        assert(len(action._joints) == len(data))
        transformed = []
        for index, joint in enumerate(action._joints):
            min_value = joint['min']
            max_value = joint['max']
            value = data[index]
            assert(value >= -1 and value <= 1)
            value = (value + 1.0) / 2.0
            value = min_value + value * (max_value - min_value)
            assert(value >= min_value)
            assert(value <= max_value)
            transformed.append(value)
        action.set_values(transformed)
        #print 'joints', action._joints
        #print 'data', data
