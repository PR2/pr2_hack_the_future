from Signal import Signal

class Action(object):

    def __init__(self):
        super(Action, self).__init__()
        self._duration = 3.0
        self.execute_finished_signal = Signal()

    def get_duration(self):
        return self._duration

    def set_duration(self, duration):
        self._duration = duration

    #@abstractmethod
    def to_string(self):
        pass

    #@abstractmethod
    def execute(self):
        pass

    def _execute_finished(self):
        self.execute_finished_signal.emit()
