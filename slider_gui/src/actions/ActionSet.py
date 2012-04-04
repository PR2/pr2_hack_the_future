from Action import Action

class ActionSet(Action):

    def __init__(self):
        super(ActionSet, self).__init__()
        self._actions = []
        self._executing = 0

    def add_action(self, action):
        self._actions.append(action)

    def remove_all_actions(self):
        self._actions = []

    def get_duration(self):
        duration = 0.0
        for action in self._actions:
            duration = max(duration, action.get_duration())
        return duration

    def set_duration(self, duration):
        for action in self._actions:
            action.set_duration(duration)

    def to_string(self):
        data = []
        for action in self._actions:
            data.append(action.to_string())
        return ';'.join(data)

    def serialize(self, stream):
        stream.serialize_data(len(self._actions))
        for action in self._actions:
            stream.serialize_instance(action)

    def deserialize(self, stream):
        self.remove_all_actions()
        count = stream.deserialize_data()
        for i in range(count):
            action = stream.deserialize_instance()
            self.add_action(action)

    def execute(self):
        if self._executing > 0:
            print('ActionSet.execute() skipped because previous execute has not yet finished')
            return
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
            self.stop()
            self._execute_finished()

    def stop(self):
        for action in self._actions:
            action.execute_finished_signal.disconnect(self._action_finished)
