#!/usr/bin/env python
import roslib
roslib.load_manifest("hack_the_web_program_executor")
roslib.load_manifest("museum_srvs")
roslib.load_manifest("slider_gui")

import rospy
import base64
import time

from museum_msgs.msg import *
from museum_srvs.srv import *
from program_queue.srv import CallProgram, CallProgramResponse




class SequenceExecutor:
        
    def __init__(self, servicename):
        self.service = rospy.ServiceProxy(servicename, PoseRobot)
        
    def execute_program(self, request):
        print "executing program!"
        sequence = PoseSequence()
        sequence.deserialize(base64.b64decode(request.code))
        poses = sequence.poses
        print "Executing %d poses" % len(poses)
        for pose in poses:
            print "calling pose service"
            self.service.call(pose)
            time.sleep(pose.duration)
        print "sequence executor finished"
        return CallProgramResponse()
            
if __name__=="__main__":
    rospy.init_node("robot_program_executor")
    
    execute_service = "/museum/run_web_slider_program"
    pose_service = "/museum/pose_robot_real"
    executor = SequenceExecutor(pose_service)
    
    execution_service = rospy.Service(execute_service, CallProgram, executor.execute_program)
    
    print "Execution service started"
    rospy.spin()