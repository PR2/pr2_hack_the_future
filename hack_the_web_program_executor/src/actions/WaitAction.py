from Action import Action

import rospy

class WaitAction(Action):

    def __init__(self, duration=1.0):
        super(WaitAction, self).__init__()
        self._timer = None
        self.set_duration(duration)

    def deepcopy(self):
        return WaitAction(self.get_duration())

    def execute(self):
        super(WaitAction, self).execute()
        print('WaitAction.execute() %d' % self.get_duration())
        self._timer = rospy.Timer(rospy.Duration.from_sec(self.get_duration()), self._timer_finished, oneshot=True)

    def _timer_finished(self, event):
        print('WaitAction.execute() finished\n')
        self._timer = None
        self._execute_finished()
