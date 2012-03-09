from Action import Action

import rospy

class WaitAction(Action):

    def __init__(self, duration):
        super(WaitAction, self).__init__()
        self._duration = duration
        self._timer = None

    def execute(self):
        super(WaitAction, self).execute()
        print('WaitAction.execute() %d' % self._duration)
        self._timer = rospy.Timer(rospy.Duration.from_sec(self._duration), self._timer_finished, oneshot=True)

    def _timer_finished(self, event):
        print('WaitAction.execute() finished\n')
        self._timer = None
        self._execute_finished()
