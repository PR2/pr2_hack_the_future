#!/usr/bin/env python

import time

import roslib
roslib.load_manifest('actionlib')
roslib.load_manifest('people_msgs')

from actionlib import SimpleActionClient
from face_detector.msg import *
from pr2_controllers_msgs.msg import *

from Action import Action
from Pr2MoveHeadAction import Pr2MoveHeadAction

import rospy

class Pr2LookAtFace(Action):

    def __init__(self):
        super(Pr2LookAtFace, self).__init__()
        self._client = SimpleActionClient('face_detector_action',FaceDetectorAction)
        self._head_client = SimpleActionClient('/head_traj_controller/point_head_action', PointHeadAction)
        self._timer = None

    def to_string(self):
        return 'look_at_face()'

    def execute(self):
        super(Pr2LookAtFace, self).execute()
        print('Pr2LookAtFace.execute() %d' % self.get_duration())
        # delay execution to not run nested in the current stack context
        self._timer = rospy.Timer(rospy.Duration.from_sec(0.001), self._execute, oneshot=True)

    def _execute(self, event):
        self._timer = rospy.Timer(rospy.Duration.from_sec(self.get_duration()), self._preempt, oneshot=True)
        connected = self._client.wait_for_server(rospy.Duration(1.0))
        if not connected:
            print('Pr2LookAtFace.execute() does nothing since face detector action was not found')
            return
        fgoal = FaceDetectorGoal()
        self._client.send_goal(fgoal)
        self._client.wait_for_result()
        f = self._client.get_result()
        if len(f.face_positions) > 0:
            closest = -1
            closest_dist = 1000
            for i in range(len(f.face_positions)):
                dist = f.face_positions[i].pos.x*f.face_positions[i].pos.x + f.face_positions[i].pos.y*f.face_positions[i].pos.y + f.face_positions[i].pos.z*f.face_positions[i].pos.z
                if dist < closest_dist: 
                    closest = i
                    closest_dist = dist
            if closest > -1:
                g = PointHeadGoal()
                g.target.header.frame_id = f.face_positions[closest].header.frame_id
                g.target.point.x = f.face_positions[closest].pos.x
                g.target.point.y = f.face_positions[closest].pos.y
                g.target.point.z = f.face_positions[closest].pos.z
                g.min_duration = rospy.Duration(1.0)
                connected = self._head_client.wait_for_server(rospy.Duration(1.0))
                if connected:
                    self._head_client.send_goal(g)
        self._finished_finding_face()

    def _preempt(self, event):
        self._client.cancel_goal()
        self._execute_finished()

    def _finished_finding_face(self):
        self._timer.shutdown()
        self._execute_finished()
