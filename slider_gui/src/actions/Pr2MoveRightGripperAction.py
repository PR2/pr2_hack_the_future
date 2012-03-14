from Pr2GripperAction import Pr2GripperAction

class Pr2MoveRightGripperAction(Pr2GripperAction):

    def __init__(self):
        super(Pr2MoveRightGripperAction, self).__init__('r_gripper_controller/command')

    def to_string(self):
        str = super(Pr2MoveRightGripperAction, self).to_string()
        return 'rgrip(%s)' % str
