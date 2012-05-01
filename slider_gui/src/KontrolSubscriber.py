import math

from actions.ActionSet import ActionSet
from actions.Pr2GripperAction import Pr2GripperAction
from actions.Pr2JointTrajectoryAction import Pr2JointTrajectoryAction
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

    previous_button = 'previous'
    play_button = 'play'
    next_button = 'next'
    repeat_button = 'repeat'
    stop_button = 'stop'
    record_button = 'record'

    top1_button = 'top1'
    bottom1_button = 'bottom1'
    top2_button = 'top2'
    bottom2_button = 'bottom2'

    top6_button = 'top6'
    bottom6_button = 'bottom6'
    top7_button = 'top7'
    bottom7_button = 'bottom7'

    top8_button = 'top8'
    bottom8_button = 'bottom8'
    top9_button = 'top9'
    bottom9_button = 'bottom9'

    def __init__(self):
        self.axes_changed = Signal()
        self.buttons_changed = Signal()
        self._axes = None
        self._buttons = None
        self._pressed_buttons = set()
        self._last_larm = None
        self._last_rarm = None
        rospy.Subscriber('korg_joy', Joy, self._joy_callback)

    def _joy_callback(self, joy_msg):
        if self._axes != joy_msg.axes:
            self._axes = joy_msg.axes
            self.axes_changed.emit()
        if self._buttons != joy_msg.buttons:
            self._buttons = joy_msg.buttons
            self.buttons_changed.emit()

    def get_action_set(self):
        set = ActionSet()
        if self._axes is None or self._buttons is None:
            head = Pr2MoveHeadAction()
            set.add_action(head)
            return set
        if len(self._buttons) >= 25:
            mode = self._buttons[24]
            if mode != 3:
                head = Pr2MoveHeadAction()
                head_data = [-self._axes[9], -self._axes[0]]
                self._set_transformed_data(head, head_data)
                set.add_action(head)

                torso = Pr2MoveTorsoAction()
                torso_data = [self._axes[8]]
                self._set_transformed_data(torso, torso_data)
                set.add_action(torso)

                lgrip = Pr2MoveLeftGripperAction()
                lgrip_data = [self._axes[7]]
                self._set_transformed_data(lgrip, lgrip_data)
                set.add_action(lgrip)

                rgrip = Pr2MoveRightGripperAction()
                rgrip_data = lgrip_data
                self._set_transformed_data(rgrip, rgrip_data)
                set.add_action(rgrip)

                rarm_data = []
                rarm_data.extend(self._axes[1:7])
                rarm_data.append(self._axes[16])
                rarm_data[1] = -rarm_data[1]

            if mode == 0 or mode == 2:
                rarm = Pr2MoveRightArmAction()
                self._set_transformed_data(rarm, rarm_data)
                set.add_action(rarm)
                self._last_rarm = rarm
            elif mode == 1 and self._last_rarm is not None:
                set.add_action(self._last_rarm.deepcopy())

            if mode != 3:
                larm_data = []
                larm_data.extend(self._axes[1:7])
                larm_data.append(self._axes[16])
                larm_data[0] = -larm_data[0]
                larm_data[1] = -larm_data[1]
                larm_data[2] = -larm_data[2]
                larm_data[4] = -larm_data[4]
                larm_data[6] = -larm_data[6]

            if mode == 0 or mode == 1:
                larm = Pr2MoveLeftArmAction()
                self._set_transformed_data(larm, larm_data)
                set.add_action(larm)
                self._last_larm = larm
            elif mode == 2 and self._last_larm is not None:
                set.add_action(self._last_larm.deepcopy())

        duration = self._transform_value(self._axes[17], 0.5, 5.0)
        set.set_duration(duration)

        return set

    def get_joint_values(self):
        values = {}
        set = self.get_action_set()
        for action in set._actions:
            is_degree = isinstance(action, Pr2JointTrajectoryAction) and not isinstance(action, Pr2MoveTorsoAction)
            for index, data in enumerate(action._joints):
                name = data['label']
                value = action._values[index]
                if is_degree:
                    value = value / 180 * math.pi
                values[name] = value
        return values

    def get_triggered_buttons(self):
        buttons = {
            18: self.previous_button,
            19: self.play_button,
            20: self.next_button,
            21: self.repeat_button,
            22: self.stop_button,
            23: self.record_button,
            0: self.top1_button,
            1: self.bottom1_button,
            2: self.top2_button,
            3: self.bottom2_button,
            10: self.top6_button,
            11: self.bottom6_button,
            12: self.top7_button,
            13: self.bottom7_button,
            14: self.top8_button,
            15: self.bottom8_button,
            16: self.top9_button,
            17: self.bottom9_button,
        }
        triggered = set()
        for index, value in enumerate(self._buttons):
            if index in buttons.keys():
                index = buttons[index]
            if value == 1:
                if index not in self._pressed_buttons:
                    self._pressed_buttons.add(index)
                    triggered.add(index)
            else:
                if index in self._pressed_buttons:
                    self._pressed_buttons.remove(index)
        return triggered

    def _set_transformed_data(self, action, data):
        assert(len(action._joints) == len(data))
        transformed = []
        for index, joint in enumerate(action._joints):
            min_value = joint['min']
            max_value = joint['max']
            value = self._transform_value(data[index], min_value, max_value)
            transformed.append(value)
        action.set_values(transformed)
        #print 'joints', action._joints
        #print 'data', data

    def _transform_value(self, value, min_value, max_value):
        assert(value >= -1 and value <= 1)
        value = (value + 1.0) / 2.0
        value = min_value + value * (max_value - min_value)
        assert(value >= min_value)
        assert(value <= max_value)
        return value
