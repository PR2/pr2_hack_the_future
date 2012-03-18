from Pr2GripperAction import Pr2GripperAction

class Pr2MoveLeftGripperAction(Pr2GripperAction):

    def __init__(self):
        super(Pr2MoveLeftGripperAction, self).__init__('l_gripper_controller/command')

    def to_string(self):
        str = super(Pr2MoveLeftGripperAction, self).to_string()
        return 'lgrip(%s)' % str
