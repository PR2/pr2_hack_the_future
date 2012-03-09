from Signal import Signal

class ActionSequence(object):

    execute_single_finished_signal = Signal()
    execute_sequence_finished_signal = Signal()

    def __init__(self):
        super(ActionSequence, self).__init__()
        self._actions = []
        self._current_action = None

    def actions(self):
        return self._actions

    def add_action(self, action):
        self._actions.append(action)

    def execute_single(self, index):
        self._execute(index, self._execute_single_finished)

    def execute_all(self, first_index = None):
        if first_index is None:
            first_index = 0
        self._execute(first_index, self._execute_sequence_finished)

    def _execute_single_finished(self):
        index = self._current_action
        self._execute_finished(self._execute_single_finished)
        self.execute_single_finished_signal.emit(index)

    def _execute_sequence_finished(self):
        index = self._current_action
        self._execute_finished(self._execute_sequence_finished)
        index += 1
        if index < len(self._actions):
            self.execute_all(index)
        else:
            self.execute_sequence_finished_signal.emit()

    def _execute(self, index, callback):
        assert(index >= 0 and index < len(self._actions))
        self._current_action = index
        action = self._actions[self._current_action]
        action.execute_finished_signal.connect(callback)
        action.execute()

    def _execute_finished(self, callback):
        index = self._current_action
        self._current_action = None

        action = self._actions[index]
        action.execute_finished_signal.disconnect(callback)
