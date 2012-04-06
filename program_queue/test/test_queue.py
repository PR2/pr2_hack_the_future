#!/usr/bin/env python
# A script to call the program queue to test that it works
#
# This script creates users and programs; as such, it expects to be run
#  against a clean database
#
# Author: Austin Hendrix

import roslib; roslib.load_manifest('program_queue')
import rospy

from program_queue.srv import *
from program_queue.msg import *

def client(name, t):
   rospy.wait_for_service(name)
   return rospy.ServiceProxy(name, t)

if __name__ == '__main__':
   rospy.init_node('test_queue')
   
   # create a user
   user = client('create_user', CreateUser)('testuser', 'testpw')
   token = user.token

   if token == 0:
      print "Failed to create user"

   # test that the token from user creation works
   program = client('create_program', CreateProgram)(token)
   program_id = program.id
   print "Program ID: %d"%program_id

   # test that we can update our program
   client('update_program', UpdateProgram)(token, Program(ProgramInfo(
      program_id, 'testprogram', ProgramInfo.PYTHON, 'testuser'), 'print "Hello World"\n'))

   # test that logout works
   client('logout', Logout)(token)

   # test that logging back in works
   user = client('login', Login)('testuser', 'testpw')
   token = user.token

   if token == 0:
      print "Failed to log in"

   if user.is_admin:
      print "Test user is admin and shouldn't be"

   # test that we can retrieve our program
   program = client('get_program', GetProgram)(program_id)
   if program.program.info.id != program_id:
      print "Failed to retrieve program"

   if program.program.info.name != 'testprogram':
      print "Failed to set program name"

   if program.program.info.type != ProgramInfo.PYTHON:
      print "Failed to set program type"

   if program.program.code != 'print "Hello World"\n':
      print "Failed to set program code"

   # test that we can queue our program
   q = client('queue_program', QueueProgram)(token, program_id)
   print "Queue position %d"%q.queue_position
   queue_position = q.queue_position

   # test that our program is in the queue
   q = client('get_queue', GetQueue)()
   if not program_id in [ p.id for p in q.programs ]:
      print "Failed to put program into queue"

   # test that our program is in the right place in the queue
   if q.programs[queue_position].id != program_id:
      print "Queue position does not match returned queue"


   # test dequeue
   client('dequeue_program', DequeueProgram)(token, program_id)

   # TODO: test that dequeue was successful

   programs = client('get_my_programs', GetMyPrograms)(token)

   # validate that the test user has one program
   if len(programs.programs) != 1:
      print "GetMyPrograms expected one program; got %d"%len(programs.programs)


   # test login as admin
   admin = client('login', Login)('admin', 'admin')
   if admin.token == 0:
      print "Failed to log in as admin"

   if not admin.is_admin:
      print "Admin user should be an admin but isn't"

   # run_program
   client('run_program', RunProgram)(admin.token, program_id)

   # get_output
   output = client('get_output', GetOutput)(token, program_id, 0)

   if len(output.output) != 1:
      print "Expected 1 output; got %d"%len(output.output)
   else:
      print "Run date: " + str(output.output[0].header.stamp.to_sec())
      print "Output: %s"%output.output[0].output


   # test get_programs
   programs = client('get_programs', GetPrograms)()

   if len(programs.programs) != 1:
      print "Expected 1 program, got %d"%len(programs.programs)

   # TODO:
   # * services to test as admin:
   #  * clear_queue
   #  * start_queue
   #  * stop_queue
