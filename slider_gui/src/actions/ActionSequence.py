import pickle
from Signal import Signal

class ActionSequence(object):

    execute_single_finished_signal = Signal()
    execute_sequence_finished_signal = Signal()

    executing_action_signal = Signal()

    def __init__(self):
        super(ActionSequence, self).__init__()
        self._actions = []
        self._current_action = None

    def actions(self):
        return self._actions

    def add_action(self, action, index = None):
        if index is None:
            self._actions.append(action)
        else:
            assert(index >=0 and index < len(self._actions))
            self._actions.insert(index, action)

    def remove_action(self, index):
        assert(index >=0 and index < len(self._actions))
        del self._actions[index]
        if self._current_action >= index:
            self._current_action = self._current_action - 1

    def remove_all_actions(self):
        self._actions = []
        self._current_action = None

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

    def execute_single(self, index):
        if self._current_action is not None:
            print('ActionSequence.execute_single() skipped because previous execute has not yet finished')
            return
        self._execute(index, self._execute_single_finished)

    def execute_all(self, first_index = None):
        if self._current_action is not None:
            print('ActionSequence.execute_all() skipped because previous execute has not yet finished')
            return
        if first_index is None:
            first_index = 0
        self._execute(first_index, self._execute_sequence_finished)

    def stop(self):
        if self._current_action is not None:
            action = self._actions[self._current_action]
            action.stop()
            self._current_action = None

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
        self.executing_action_signal.emit(self._current_action)
        action.execute_finished_signal.connect(callback)
        action.execute()

    def _execute_finished(self, callback):
        index = self._current_action
        self._current_action = None

        action = self._actions[index]
        action.execute_finished_signal.disconnect(callback)
