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

    def get_value(self, label):
        for action in self._actions:
            try:
                value = action.get_value(label)
                return value
            except AttributeError:
                # action does not support get_value
                pass
            except KeyError:
                # action does not have a joint with that label
                pass
        raise KeyError('joint with label "%s" not found' % label)

    def get_joint_info(self, label):
        for action in self._actions:
            try:
                indexes = [i for i, data in enumerate(action._joints) if data['label'] == label]
                if len(indexes) == 1:
                    return action._joints[indexes[0]]
            except AttributeError:
                # action does not support get_value
                pass
        raise KeyError('joint with label "%s" not found' % label)

    def update_value(self, label, value):
        for action in self._actions:
            try:
                action.update_value(label, value)
                return
            except:
                pass
        raise KeyError('joint with label "%s" not found' % label)

    def to_string(self):
        data = []
        for action in self._actions:
            data.append(action.to_string())
        return ';'.join(data)

    def deepcopy(self):
        set = super(ActionSet, self).deepcopy()
        for action in self._actions:
            set.add_action(action.deepcopy())
        return set

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
        #print('ActionSet.execute() %d' % len(self._actions))
        self._executing = len(self._actions)
        for action in self._actions:
            action.execute_finished_signal.connect(self._action_finished)
            action.execute()

    def _action_finished(self):
        assert(self._executing > 0)
        self._executing -= 1
        if self._executing == 0:
            #print('ActionSet.execute() finished\n')
            self.stop()
            self._execute_finished()

    def stop(self):
        print('ActionSet.stop()\n')
        for action in self._actions:
            action.execute_finished_signal.disconnect(self._action_finished)
        self._executing = 0
