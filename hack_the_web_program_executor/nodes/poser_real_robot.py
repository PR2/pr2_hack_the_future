#!/usr/bin/env python
import roslib
import random
roslib.load_manifest("robot_poser")
roslib.load_manifest("museum_srvs")

import rospy
import sys

import poseutils

from museum_srvs.srv import PoseRobot, PoseRobotResponse

""" This node advertises a service to pose the robot """



class Poser():
    
    def __init__(self, servicename):
        self.service = rospy.Service(servicename, PoseRobot, self.pose_robot_cb)
        
    def pose_robot_cb(self, request):
        self.pose_robot(request.pose)
        return PoseRobotResponse()
    
    def pose_robot(self, pose):
        actionset = poseutils.scaled_pose_to_actionset(pose)
        actionset.execute()
        
if __name__=="__main__":
    
    pose_service = "/museum/pose_robot_real"
    
    rospy.init_node("robot_poser_real")
    
    p = Poser(pose_service)
    
    print "Poser service started"
    
    rospy.spin()