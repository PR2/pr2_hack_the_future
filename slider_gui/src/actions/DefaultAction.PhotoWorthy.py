from actions.Pr2MoveHeadAction import Pr2MoveHeadAction
from actions.Pr2MoveLeftArmAction import Pr2MoveLeftArmAction
from actions.Pr2MoveLeftGripperAction import Pr2MoveLeftGripperAction
from actions.Pr2MoveRightArmAction import Pr2MoveRightArmAction
from actions.Pr2MoveRightGripperAction import Pr2MoveRightGripperAction
from actions.Pr2MoveTorsoAction import Pr2MoveTorsoAction
from ActionSet import ActionSet

class DefaultAction(ActionSet):

    def __init__(self):
        super(DefaultAction, self).__init__()

        head = Pr2MoveHeadAction()
        values = head.values()
        values[1] = 0
        head.set_values(values)
        self.add_action(head)

        torso = Pr2MoveTorsoAction()
        self.add_action(torso)

        lgrip = Pr2MoveLeftGripperAction()
        self.add_action(lgrip)

        rgrip = Pr2MoveRightGripperAction()
        self.add_action(rgrip)

        rarm = Pr2MoveRightArmAction()
        rvalues = rarm.values()
        rvalues[0] = -85
        rvalues[2] = -190
        rvalues[3] = -90
        rarm.set_values(rvalues)
        self.add_action(rarm)

        larm = Pr2MoveLeftArmAction()
        lvalues = larm.values()
        lvalues[0] = -rvalues[0]
        lvalues[2] = -rvalues[2]
        lvalues[3] = rvalues[3]
        larm.set_values(lvalues)
        self.add_action(larm)
