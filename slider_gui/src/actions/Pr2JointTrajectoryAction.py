import roslib
roslib.load_manifest('actionlib')
roslib.load_manifest('pr2_controllers_msgs')

from Action import Action
from actionlib import SimpleActionClient
import math
from pr2_controllers_msgs.msg import JointTrajectoryAction, JointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectoryPoint
import rospy

class Pr2JointTrajectoryAction(Action):

    def __init__(self, topic, transform_to_radian = True):
        super(Pr2JointTrajectoryAction, self).__init__()
        self._client = SimpleActionClient(topic, JointTrajectoryAction)
        self._joints = []
        self._values = []
        self._transform_to_radian = transform_to_radian
        self._duration = 3.0
        self._timer = None

    def set_values(self, values):
        assert(len(self._values) == len(values))
        for i, desc in enumerate(self._joints):
            # clamp values to valid range
            self._values[i] = max(desc['min'], min(values[i], desc['max']))

    def to_string(self):
        data = []
        for value in self._values:
            data.append('%.1f' % value)
        return ','.join(data)

    def _add_joint(self, label, min, max):
        self._joints.append({'label': label, 'min': min, 'max': max})
        # default value between min and max
        self._values.append((max - min) / 2.0)

    def execute(self):
        super(Pr2JointTrajectoryAction, self).execute()
        goal = JointTrajectoryGoal()
        point = JointTrajectoryPoint()
        goal.trajectory.joint_names = [''] * len(self._joints)
        point.positions = [0.0] * len(self._joints)
        point.velocities = [0.0] * len(self._joints)
        for i, desc in enumerate(self._joints):
            goal.trajectory.joint_names[i] = desc['label']
            point.positions[i] = self._values[i]
            if self._transform_to_radian:
                point.positions[i] *= math.pi / 180.0
        point.time_from_start = rospy.Duration.from_sec(self._duration)
        goal.trajectory.points = [ point ]
        print('Pr2JointTrajectoryAction.execute() %s: %s' % (self.__class__.__name__, ','.join([str(value) for value in self._values])))
        self._client.send_goal(goal)
        self._timer = rospy.Timer(rospy.Duration.from_sec(self._duration), self._timer_finished, oneshot=True)

    def _timer_finished(self, event):
        print('Pr2JointTrajectoryAction.execute() finished %s\n' % (self.__class__.__name__))
        self._timer = None
        self._execute_finished()
