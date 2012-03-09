import roslib
roslib.load_manifest('pr2_controllers_msgs')

from Action import Action
from pr2_controllers_msgs.msg import Pr2GripperCommand
import rospy

class Pr2GripperAction(Action):

    def __init__(self, topic):
        super(Pr2GripperAction, self).__init__()
        self._pub = rospy.Publisher(topic, Pr2GripperCommand)
        self._min_value = 0.0
        self._max_value = 0.08
        self._duration = 3.0
        self._value = (self._max_value- self._min_value) / 2.0
        self._timer = None

    def set_value(self, value):
        # clamp value to valid range
        self._value = max(self._min_value, min(value, self._max_value))

    def execute(self):
        super(Pr2GripperAction, self).execute()
        command = Pr2GripperCommand(0.04, 0)
        command.max_effort = 10.0
        command.position = self._value
        print('Pr2GripperAction.execute() %s: %s' % (self.__class__.__name__, str(self._value)))
        self._pub.publish(command)
        self._timer = rospy.Timer(rospy.Duration.from_sec(self._duration), self._timer_finished, oneshot=True)

    def _timer_finished(self, event):
        print('Pr2GripperAction.execute() finished %s\n' % (self.__class__.__name__))
        self._timer = None
        self._execute_finished()
