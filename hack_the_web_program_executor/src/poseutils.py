#!/usr/bin/env python
from actions.ActionSet import ActionSet
from actions.Pr2MoveHeadAction import Pr2MoveHeadAction
from actions.Pr2MoveLeftArmAction import Pr2MoveLeftArmAction
from actions.Pr2MoveLeftGripperAction import Pr2MoveLeftGripperAction
from actions.Pr2MoveRightArmAction import Pr2MoveRightArmAction
from actions.Pr2MoveRightGripperAction import Pr2MoveRightGripperAction
from actions.Pr2MoveTorsoAction import Pr2MoveTorsoAction

""" Contains methods to convert between Pose messages and actions """

def pose_to_actionset(pose):
    """ Takes the given museum_msgs/Pose message and converts it into an ActionSet
    
    It is assumed that pose values are between 0 and 1.  They will be scaled """
    
    head = Pr2MoveHeadAction()
    head_vals = [pose.head.head_pan, pose.head.head_tilt]
    _set_transformed_data(head, head_vals)

    torso = Pr2MoveTorsoAction()
    torso_vals = [pose.torso]
    _set_transformed_data(torso, torso_vals)
    
    def arm_data(arm):
            return [
                arm.shoulder_pan,
                arm.shoulder_lift,
                arm.upper_arm_roll,
                arm.elbow_flex,
                arm.forearm_roll,
                arm.wrist_flex,
                arm.wrist_roll
            ]
    
    larm = Pr2MoveLeftArmAction()
    larm_vals = arm_data(pose.left)
    _set_transformed_data(larm, larm_vals)

    lgrip = Pr2MoveLeftGripperAction()
    lgrip_vals = [pose.left.gripper]
    _set_transformed_data(lgrip, lgrip_vals)
    
    rarm = Pr2MoveRightArmAction()
    rarm_vals = arm_data(pose.right)
    _set_transformed_data(rarm, rarm_vals)

    rgrip = Pr2MoveRightGripperAction()
    rgrip_vals = [pose.right.gripper]
    _set_transformed_data(rgrip, rgrip_vals)
    
    actionset = ActionSet()
    actionset.add_action(head)
    actionset.add_action(torso)
    actionset.add_action(larm)
    actionset.add_action(lgrip)
    actionset.add_action(rarm)
    actionset.add_action(rgrip)
    
    actionset.set_duration(_transform_value(pose.duration, 0.5, 5.0))
                
    print actionset.to_string()
        
    return actionset



        
def scaled_pose_to_actionset(pose):
    """ Takes the given museum_msgs/Pose message and converts it into an ActionSet
    
    It is assumed that pose values are already scaled """

    head = Pr2MoveHeadAction()
    head.set_values([pose.head.head_pan, pose.head.head_tilt])

    torso = Pr2MoveTorsoAction()
    torso.set_values([pose.torso])
    
    def arm_data(arm):
        return [
            arm.shoulder_pan,
            arm.shoulder_lift,
            arm.upper_arm_roll,
            arm.elbow_flex,
            arm.forearm_roll,
            arm.wrist_flex,
            arm.wrist_roll
        ]
    
    larm = Pr2MoveLeftArmAction()
    larm.set_values(arm_data(pose.left))

    lgrip = Pr2MoveLeftGripperAction()
    lgrip.set_values([pose.left.gripper])
    
    rarm = Pr2MoveRightArmAction()
    rarm.set_values(arm_data(pose.right))

    rgrip = Pr2MoveRightGripperAction()
    rgrip.set_values([pose.right.gripper])
    
    duration = pose.duration
    
    actionset = ActionSet()
    actionset.add_action(head)
    actionset.add_action(torso)
    actionset.add_action(larm)
    actionset.add_action(lgrip)
    actionset.add_action(rarm)
    actionset.add_action(rgrip)
    actionset.set_duration(duration)
    
    return actionset
    
def _set_transformed_data(action, data):
    action.set_values(_transform_data(action, data))
    
def _transform_data(action, data):
    assert(len(action._joints) == len(data))
    transformed = []
    for index, joint in enumerate(action._joints):
        min_value = joint['min']
        max_value = joint['max']
        value = _transform_value(data[index], min_value, max_value)
        transformed.append(value)
    return transformed    

def _transform_value(value, min_value, max_value):
    assert(value >= -1 and value <= 1)
    value = (value + 1.0) / 2.0
    value = min_value + value * (max_value - min_value)
    assert(value >= min_value)
    assert(value <= max_value)
    return value