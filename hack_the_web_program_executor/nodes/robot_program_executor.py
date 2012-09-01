#!/usr/bin/env python
import roslib
roslib.load_manifest("hack_the_web_program_executor")
roslib.load_manifest("museum_srvs")
roslib.load_manifest("slider_gui")

import rospy
import base64
import time
from threading import Lock, Thread, Condition

from museum_msgs.msg import *
from museum_srvs.srv import *
from sensor_msgs.msg import Joy
from program_queue.srv import CallProgram, CallProgramResponse
from actions.DefaultAction import DefaultAction


class WebProgramExecutor:
    
    lock = None
    executor = None
    previous_request = None
    
    def __init__(self):
        self.lock = Lock()
        self.execution_service = rospy.Service("/museum/run_web_slider_program", CallProgram, self.execute_program)
        self._default_pose = DefaultAction()
        self._default_pose.set_duration(8.0)
        rospy.Subscriber('ps3_joy', Joy, self._joy_callback)
    
    def execute_program(self, request):
        print "Received program execution request"
        sequence = PoseSequence()
        sequence.deserialize(base64.b64decode(request.code))
        poses = sequence.poses
        
                
        print "Executing program"
        
        with self.lock:
            if self.executor:
                self.executor.cancel()
            self.executor = None
            executor = SequenceExecutor(poses, self._default_pose)
            executor.start()
            self.executor = executor
            
        executor.join()
        
        print "Program execution finished"
        
        return CallProgramResponse()
    
    def _joy_callback(self, request):
        triggered_buttons = request.buttons
        left_pressed = (not self.previous_request and request.buttons[7]) or (request.buttons[7] and not self.previous_request.buttons[7])
        square_pressed = (not self.previous_request and request.buttons[15]) or (request.buttons[15] and not self.previous_request.buttons[15])
        if left_pressed:
            with self.lock:
                if self.executor:
                    self.executor.cancel()
        if square_pressed:
            with self.lock:
                if self.executor:
                    with self.executor.play:
                        self.executor.play.notify()
        
        
class SequenceExecutor(Thread):
    
    lock = Lock()
    active = True
    play = Condition()
        
    def __init__(self, poses, default):
        Thread.__init__(self)
        self.service = rospy.ServiceProxy("/museum/pose_robot_real", PoseRobot)
        self.poses = poses
        self.default = default
        
    def cancel(self):
        with self.lock:
            self.active = False
        with self.play:
            self.play.notify()
        
    def run(self):
        with self.lock:
            if not self.active:
                print "sequence executor interrupted"
                return
            print "Moving to default pose"
            self.default.execute()
        time.sleep(8)
        
        while True:
            with self.play:
                print "Awaiting play or cancel"
                self.play.wait()
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