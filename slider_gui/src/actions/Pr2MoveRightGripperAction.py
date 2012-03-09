from Pr2GripperAction import Pr2GripperAction

class Pr2MoveRightGripperAction(Pr2GripperAction):
    def __init__(self):
        super(Pr2MoveRightGripperAction, self).__init__('r_gripper_controller/command')
