#!/usr/bin/env python
# A script to call the program queue to test that it works
#
# Author: Austin Hendrix

import roslib; roslib.load_manifest('program_queue')
import rospy

from program_queue.srv import *

def client(name, t):
   rospy.wait_for_service(name)
   return rospy.ServiceProxy(name, t)

if __name__ == '__main__':
   rospy.init_node('test_queue')
   
   user = client('login', Login)('testuser', 'testpw')
   token = user.token
   if token == 0:
      user = client('create_user', CreateUser)('testuser', 'testpw')
      token = user.token
   print token

   program = client('create_program', CreateProgram)(token)
   print "Program ID: %d"%program.id
