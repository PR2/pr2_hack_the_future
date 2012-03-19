#!/usr/bin/env python

import roslib; roslib.load_manifest('program_queue')
import rospy

from program_queue.srv import *
from program_queue.msg import *
from std_srvs.srv import Empty


class Queue:
   def __init__(self):
      # TODO: set up database

      # TODO: set up services
      rospy.Service('clear_queue',     ClearQueue,     self.handle_clear_queue)
      rospy.Service('create_user',     CreateUser,     self.handle_create_user)
      rospy.Service('dequeue_program', DequeueProgram, 
            self.handle_dequeue_program)
      rospy.Service('get_output',      GetOutput,      self.handle_get_output)
      rospy.Service('get_program',     GetProgram,     self.handle_get_program)
      rospy.Service('get_programs',    GetPrograms,    self.handle_get_programs)
      rospy.Service('get_queue',       GetQueue,       self.handle_get_queue)
      rospy.Service('login',           Login,          self.handle_login)
      rospy.Service('logout',          Logout,         self.handle_logout)
      rospy.Service('queue_program',   QueueProgram,   
            self.handle_queue_program)
      rospy.Service('run_program',     RunProgram,     self.handle_run_program)
      rospy.Service('update_program',  UpdateProgram, 
            self.handle_update_program)

      rospy.Service('start_queue',     Empty,          self.handle_start_queue)
      rospy.Service('stop_queue',     Empty,           self.handle_stop_queue)


      rospy.loginfo("Queue ready")

   def handle_clear_queue(self, req):
      return ClearQueueResponse()

   def handle_create_user(self, req):
      return CreateUserResponse()

   def handle_dequeue_program(self, req):
      return DequeueProgramResponse()

   def handle_get_output(self, req):
      return GetOutputResponse()

   def handle_get_program(self, req):
      return GetProgramResponse()

   def handle_get_programs(self, req):
      return GetProgramsResponse()

   def handle_get_queue(self, req):
      return GetQueueResponse()

   def handle_login(self, req):
      return LoginResponse(0)

   def handle_logout(self, req):
      return LogoutResponse()

   def handle_queue_program(self, req):
      return QueueProgramResponse()

   def handle_run_program(self, req):
      return RunProgramResponse()

   def handle_start_queue(self, req):
      return EmptyResponse()

   def handle_stop_queue(self, req):
      return EmptyResponse()

   def handle_update_program(self, req):
      return UpdateProgram()


if __name__ == '__main__':
   rospy.init_node('program_queue')

   queue = Queue()

   rospy.spin()
