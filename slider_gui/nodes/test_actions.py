#!/usr/bin/env python

import roslib; roslib.load_manifest('slider_gui')

from actions.ActionSequence import ActionSequence
from actions.ActionSet import ActionSet
from actions.Pr2MoveHeadAction import Pr2MoveHeadAction
from actions.Pr2MoveLeftArmAction import Pr2MoveLeftArmAction
from actions.Pr2MoveLeftGripperAction import Pr2MoveLeftGripperAction
from actions.Pr2MoveRightArmAction import Pr2MoveRightArmAction
from actions.Pr2MoveTorsoAction import Pr2MoveTorsoAction
from actions.WaitAction import WaitAction
import rospy

rospy.init_node('test_actions')

seq = ActionSequence()

action1 = Pr2MoveLeftArmAction()
action1.set_values([0, 0, 0, 0, 0, 0, 0])
seq.add_action(action1)

action2 = Pr2MoveLeftArmAction()
action2.set_values([0, 88, 0, -90, 0, 0, 90])
seq.add_action(action2)

action3 = Pr2MoveRightArmAction()
action3.set_values([0, 0, 0, 0, 0, 0, 0])
seq.add_action(action3)

action4 = Pr2MoveRightArmAction()
action4.set_values([0, 88, 0, -90, 0, 0, 90])
seq.add_action(action4)

action5 = WaitAction(3.0)
seq.add_action(action5)

action6 = Pr2MoveLeftGripperAction()
action6.set_value(0.0)
seq.add_action(action6)

action7 = Pr2MoveLeftGripperAction()
action7.set_value(0.08)
seq.add_action(action7)

action8 = Pr2MoveHeadAction()
action8.set_values([-60, -30])
action9 = Pr2MoveTorsoAction()
action9.set_values([0.0])
set1 = ActionSet()
set1.add_action(action8)
set1.add_action(action9)
seq.add_action(set1)

action10 = Pr2MoveHeadAction()
action10.set_values([60, 30])
action11 = Pr2MoveTorsoAction()
action11.set_values([0.3])
set2 = ActionSet()
set2.add_action(action10)
set2.add_action(action11)
seq.add_action(set2)

action5 = WaitAction(3.0)
seq.add_action(action5)

def execute_sequence():
    global seq
    seq.execute_all()

seq.execute_sequence_finished_signal.connect(execute_sequence)

execute_sequence()

rospy.spin()
