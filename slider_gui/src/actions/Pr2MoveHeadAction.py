from Pr2JointTrajectoryAction import Pr2JointTrajectoryAction

class Pr2MoveHeadAction(Pr2JointTrajectoryAction):

    def __init__(self):
        super(Pr2MoveHeadAction, self).__init__('head_traj_controller/joint_trajectory_action')
        self._add_joint('head_pan_joint', -160, 160)
        self._add_joint('head_tilt_joint', -30, 90)

    def to_string(self):
        str = super(Pr2MoveHeadAction, self).to_string()
        return 'head(%s)' % str
