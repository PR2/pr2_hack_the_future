from Action import Action

class ActionSet(Action):

    def __init__(self):
        super(ActionSet, self).__init__()
        self._actions = []
        self._executing = 0

    def add_action(self, action):
        self._actions.append(action)

    def execute(self):
        super(ActionSet, self).execute()
        print('ActionSet.execute() %d' % len(self._actions))
        self._executing = len(self._actions)
        for action in self._actions:
            action.execute_finished_signal.connect(self._action_finished)
            action.execute()

    def _action_finished(self):
        assert(self._executing > 0)
        self._executing -= 1
        if self._executing == 0:
            print('ActionSet.execute() finished\n')
            for action in self._actions:
                action.execute_finished_signal.disconnect(self._action_finished)
            self._execute_finished()
