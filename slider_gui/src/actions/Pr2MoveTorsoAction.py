from Pr2JointTrajectoryAction import Pr2JointTrajectoryAction

class Pr2MoveTorsoAction(Pr2JointTrajectoryAction):

    def __init__(self):
        super(Pr2MoveTorsoAction, self).__init__('torso_controller/joint_trajectory_action', False)
        self._add_joint('torso_lift_joint', 0, 0.3, 0.01)

    def to_string(self):
        return 'torso(%.0f)' % (100 * self._values[0])
