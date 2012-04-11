import rospy
from sensor_msgs.msg import Joy
from Signal import Signal

class Ps3Subscriber(object):

    top_button = 'top'
    right_button = 'right'
    bottom_button = 'bottom'
    left_button = 'left'

    select_button = 'select'
    start_button = 'start'

    square_button = 'square'
    triangle_button = 'triangle'
    cross_button = 'cross'
    circle_button = 'circle'

    def __init__(self):
        self.buttons_changed = Signal()
        self._buttons = None
        self._pressed_buttons = set()
        rospy.Subscriber('ps3_joy', Joy, self._joy_callback)

    def _joy_callback(self, joy_msg):
        if self._buttons != joy_msg.buttons:
            self._buttons = joy_msg.buttons
            self.buttons_changed.emit()

    def get_triggered_buttons(self):
        buttons = {
            4: self.top_button,
            5: self.right_button,
            6: self.bottom_button,
            7: self.left_button,
            0: self.select_button,
            3: self.start_button,
            15: self.square_button,
            12: self.triangle_button,
            14: self.cross_button,
            13: self.circle_button,
        }
        triggered = set()
        for index, value in enumerate(self._buttons):
            if index in buttons.keys():
                index = buttons[index]
            if value != 0:
                if index not in self._pressed_buttons:
                    self._pressed_buttons.add(index)
                    triggered.add(index)
            else:
                if index in self._pressed_buttons:
                    self._pressed_buttons.remove(index)
        return triggered
