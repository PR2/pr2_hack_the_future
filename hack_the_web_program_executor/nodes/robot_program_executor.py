#!/usr/bin/env python
import roslib
roslib.load_manifest("hack_the_web_program_executor")
roslib.load_manifest("museum_srvs")
roslib.load_manifest("slider_gui")

import rospy
import base64
import time
from threading import Lock, Thread

from museum_msgs.msg import *
from museum_srvs.srv import *
from program_queue.srv import CallProgram, CallProgramResponse
from actions.DefaultAction import DefaultAction
from Ps3Subscriber import Ps3Subscriber

class WebProgramExecutor:
    
    lock = None
    executor = None
    
    def __init__(self):
        self.lock = Lock()
        self.execution_service = rospy.Service("/museum/run_web_slider_program", CallProgram, self.execute_program)
        self._default_pose = DefaultAction()
        self._default_pose.set_duration(8.0)
    
    def execute_program(self, request):
        print "Received program execution request"
        sequence = PoseSequence()
        sequence.deserialize(base64.b64decode(request.code))
        poses = sequence.poses
        
        with self.lock:
            if self.executor:
                self.executor.cancel()
            executor = SequenceExecutor(poses, self._default_pose)
            executor.start()
            self.executor = executor
            
        executor.join()
        
        print "Program execution finished"
        
        return CallProgramResponse()
        
class SequenceExecutor(Thread):
    
    lock = Lock()
    active = True
        
    def __init__(self, poses, default):
        Thread.__init__(self)
        self.service = rospy.ServiceProxy("/museum/pose_robot_real", PoseRobot)
        self.poses = poses
        self.default = default
        
    def cancel(self):
        with self.lock:
            self.active = False
        
    def run(self):
        with self.lock:
            if not self.active:
                print "sequence executor interrupted"
                return
            print "Moving to default pose"
            self.default.execute()
        time.sleep(8)
        poses = self.poses
        print "Executing %d poses" % len(poses)
        for pose in poses:
            print "Posing robot..."
            with self.lock:
                if not self.active:
                    print "sequence executor interrupted"
                    return
                self.service.call(pose)
            time.sleep(pose.duration)
            
if __name__=="__main__":
    rospy.init_node("robot_program_executor")
    
    executor = WebProgramExecutor()
    
    print "Execution service started"
    rospy.spin()