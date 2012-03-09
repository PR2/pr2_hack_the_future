from Pr2JointTrajectoryAction import Pr2JointTrajectoryAction

class Pr2MoveRightArmAction(Pr2JointTrajectoryAction):
    def __init__(self):
        super(Pr2MoveRightArmAction, self).__init__('r_arm_controller/joint_trajectory_action')
        self._add_joint('r_shoulder_pan_joint', -130, 0)
        self._add_joint('r_shoulder_lift_joint', -30, 80)
        self._add_joint('r_upper_arm_roll_joint', -224, 44)
        self._add_joint('r_elbow_flex_joint', -130, 0)
        self._add_joint('r_forearm_roll_joint', -180, 180)
        self._add_joint('r_wrist_flex_joint', -130, 0)
        self._add_joint('r_wrist_roll_joint', -180, 180)
