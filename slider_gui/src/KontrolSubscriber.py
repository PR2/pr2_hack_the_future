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

    def __init__(self):
        self.axes_changed = Signal()
        self.buttons_changed = Signal()
        self._axes = None
        self._buttons = None
        self._pressed_buttons = set()
        rospy.Subscriber('joy', Joy, self._joy_callback)

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
            if mode == 0:
                head = Pr2MoveHeadAction()
                head_data = [-self._axes[9], -self._axes[0]]
                self._set_transformed_data(head, head_data)
                set.add_action(head)

                torso = Pr2MoveTorsoAction()
                torso_data = [self._axes[1]]
                self._set_transformed_data(torso, torso_data)
                set.add_action(torso)

                lgrip = Pr2MoveLeftGripperAction()
                lgrip_data = [self._axes[8]]
                self._set_transformed_data(lgrip, lgrip_data)
                set.add_action(lgrip)

                rgrip = Pr2MoveRightGripperAction()
                rgrip_data = lgrip_data
                self._set_transformed_data(rgrip, rgrip_data)
                set.add_action(rgrip)

                rarm_data = []
                rarm_data.extend(self._axes[2:8])
                rarm_data.append(self._axes[17])
                rarm_data[1] = -rarm_data[1]

                rarm = Pr2MoveRightArmAction()
                self._set_transformed_data(rarm, rarm_data)
                set.add_action(rarm)

                larm_data = []
                larm_data.extend(self._axes[2:8])
                larm_data.append(self._axes[17])
                larm_data[0] = -larm_data[0]
                larm_data[1] = -larm_data[1]
                larm_data[2] = -larm_data[2]
                larm_data[4] = -larm_data[4]
                larm_data[6] = -larm_data[6]

                larm = Pr2MoveLeftArmAction()
                self._set_transformed_data(larm, larm_data)
                set.add_action(larm)

        return set

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
