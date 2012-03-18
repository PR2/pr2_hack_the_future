from Signal import Signal

class Action(object):

    def __init__(self):
        super(Action, self).__init__()
        self.execute_finished_signal = Signal()

    #@abstractmethod
    def to_string(self):
        pass

    #@abstractmethod
    def execute(self):
        pass

    def _execute_finished(self):
        self.execute_finished_signal.emit()
