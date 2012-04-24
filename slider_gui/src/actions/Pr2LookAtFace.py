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

    def __init__(self):#, duration = 3.0):
        super(Pr2LookAtFace, self).__init__()
        self._client = SimpleActionClient('face_detector_action',FaceDetectorAction)
        self._client.wait_for_server()
        self._timer = None
        #self.set_duration(duration)

    def to_string(self):
        #str = super(Pr2LookAtFace, self).to_string()
        return 'look_for_face()'# % str

#    def deepcopy(self):
#        return Pr2LookAtFace(self.get_duration())

    def execute(self):
        super(Pr2LookAtFace, self).execute()
        print('Pr2LookAtFace.execute() %d' % self.get_duration())
        self._timer = rospy.Timer(rospy.Duration.from_sec(self.get_duration()), self._preempt, oneshot=True)
        fgoal = FaceDetectorGoal()
        self._client.send_goal(fgoal)
        self._client.wait_for_result()
        f = self._client.get_result()
        if len(f.face_positions) > 0:
            g = PointHeadGoal()
            g.target.header.frame_id = f.face_positions[0].header.frame_id
            g.target.point.x = f.face_positions[0].pos.x
            g.target.point.y = f.face_positions[0].pos.y
            g.target.point.z = f.face_positions[0].pos.z
            g.min_duration = rospy.Duration(1.0)
            head_client.send_goal(g)
        self._finished_finding_face()

    def _preempt(self, event):
        self._client.cancel_goal()
        self._execute_finished()

    def _finished_finding_face(self):
        self._timer.shutdown()
        self._execute_finished()

    def _timer_finished(self, event):
        self._timer = None
        self._execute_finished()
            
    
