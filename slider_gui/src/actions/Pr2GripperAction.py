import copy

import roslib
roslib.load_manifest('pr2_controllers_msgs')

from Action import Action
from pr2_controllers_msgs.msg import Pr2GripperCommand
import rospy

class Pr2GripperAction(Action):

    def __init__(self, topic, label):
        super(Pr2GripperAction, self).__init__()
        self._pub = rospy.Publisher(topic, Pr2GripperCommand)
        self._min_value = 0.0
        self._max_value = 0.08
        self._joints = [{'label': label, 'min': self._min_value, 'max': self._max_value, 'single_step': 0.005}]
        self._values = [(self._max_value- self._min_value) / 2.0]
        self._timer = None

    def set_values(self, values):
        # clamp value to valid range
        assert(len(values) == 1)
        self._values = [max(self._min_value, min(values[0], self._max_value))]

    def get_value(self, label):
        indexes = [i for i, data in enumerate(self._joints) if data['label'] == label]
        if len(indexes) == 1:
            return self._values[indexes[0]]
        raise KeyError('joint with label "%s" not found' % label)

    def update_value(self, label, value):
        indexes = [i for i, data in enumerate(self._joints) if data['label'] == label]
        if len(indexes) == 1:
            self._values[indexes[0]] = value
        raise KeyError('joint with label "%s" not found' % label)

    def to_string(self):
        return '%.1f' % (100 * self._values[0]) 

    def deepcopy(self):
        action = super(Pr2GripperAction, self).deepcopy()
        action._values = copy.deepcopy(self._values)
        return action

    def serialize(self, stream):
        super(Pr2GripperAction, self).serialize(stream)
        stream.serialize_data(self._values)

    def deserialize(self, stream):
        super(Pr2GripperAction, self).deserialize(stream)
        self._values = stream.deserialize_data()

    def execute(self):
        super(Pr2GripperAction, self).execute()
        command = Pr2GripperCommand(0.04, 0)
        command.max_effort = 20.0
        command.position = self._values[0]
        #print('Pr2GripperAction.execute() %s: %s' % (self.__class__.__name__, str(self._values[0])))
        self._pub.publish(command)
        self._timer = rospy.Timer(rospy.Duration.from_sec(self.get_duration()), self._timer_finished, oneshot=True)

    def _timer_finished(self, event):
        #print('Pr2GripperAction.execute() finished %s\n' % (self.__class__.__name__))
        self._timer = None
        self._execute_finished()
