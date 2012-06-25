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
        valuesh = head.values()
        valuesh[0] = 0
        valuesh[1] = -2
        head.set_values(valuesh)
        self.add_action(head)

        torso = Pr2MoveTorsoAction()
        valuest = torso.values()
        valuest[0] = 0
        torso.set_values(valuest)
        self.add_action(torso)

        lgrip = Pr2MoveLeftGripperAction()
        #valuesg = lgrip._values
        valuesg = [0.04]
        lgrip.set_values(valuesg)
        self.add_action(lgrip)

        rgrip = Pr2MoveRightGripperAction()
        #valuesg = rgrip.values
        valuesg = [0.04]
        rgrip.set_values(valuesg)
        self.add_action(rgrip)

        larm = Pr2MoveLeftArmAction()
        lvalues = larm.values()
        lvalues[0] = 35
        lvalues[1] = 63
        lvalues[2] = 122
        lvalues[3] = -106
        lvalues[4] = -60
        lvalues[5] = 0
        lvalues[6] = 0
        larm.set_values(lvalues)
        self.add_action(larm)

        rarm = Pr2MoveRightArmAction()
        rvalues = rarm.values()
        rvalues[0] = -72
        rvalues[1] = 74
        rvalues[2] = -62
        rvalues[3] = -60
        rvalues[4] = -3
        rvalues[5] = 0
        rvalues[6] = 0
        rarm.set_values(rvalues)
        self.add_action(rarm)
