from Pr2JointTrajectoryAction import Pr2JointTrajectoryAction

class Pr2MoveLeftArmAction(Pr2JointTrajectoryAction):

    def __init__(self):
        super(Pr2MoveLeftArmAction, self).__init__('l_arm_controller/joint_trajectory_action')
        self._add_joint('l_shoulder_pan_joint', 0, 130)
        self._add_joint('l_shoulder_lift_joint', -30, 80)
        self._add_joint('l_upper_arm_roll_joint', -44, 224)
        self._add_joint('l_elbow_flex_joint', -130, 0)
        self._add_joint('l_forearm_roll_joint', -180, 180)
        self._add_joint('l_wrist_flex_joint', -130, 0)
        self._add_joint('l_wrist_roll_joint', -180, 180)

    def to_string(self):
        str = super(Pr2MoveLeftArmAction, self).to_string()
        return 'larm(%s)' % str
