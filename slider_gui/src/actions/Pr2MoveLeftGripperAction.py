from Pr2GripperAction import Pr2GripperAction

class Pr2MoveLeftGripperAction(Pr2GripperAction):
    def __init__(self):
        super(Pr2MoveLeftGripperAction, self).__init__('l_gripper_controller/command')
