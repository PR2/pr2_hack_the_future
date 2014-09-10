import roslib;
roslib.load_manifest('program_queue')
roslib.load_manifest('slider_gui')
import rospy;

import program_queue.msg
import program_queue.srv

from slider_gui.srv import *

class ProgramQueue():

    def __init__(self, username, password):
        self._username = username
        self._password = password
        self._token = None
        self._next_serial = 1

    def login(self):
        # log in
        print 'logging in...'
        user = self._client('login', program_queue.srv.Login)(self._username, self._password)
        token = user.token
        if token == 0:
            print 'Failed to log in'
            self._token = None
            return False
        self._token = token
        return True

    def logout(self):
        # log out
        print 'logging out...'
        self._client('logout', program_queue.srv.Logout)(self._token)
        self._token = None

    def upload_program(self, program_data, label=None):
        if self._token is None:
            return False

        if label is None:
            label = 'program%d' % self._next_serial
            self._next_serial += 1
        print 'ProgramQueue.upload_program() %s' % label

        # create empty program
        print 'creating program...'
        program = self._client('create_program', program_queue.srv.CreateProgram)(self._token)
        program_id = program.id

        # upload program data
        print 'uploading program...'
        program_info = program_queue.msg.ProgramInfo(program_id, label, program_queue.msg.ProgramInfo.SLIDER, self._username)
        program = program_queue.msg.Program(program_info, program_data)
        self._client('update_program', program_queue.srv.UpdateProgram)(self._token, program)

        # add to queue
        print 'adding to queue...'
        self._client('queue_program', program_queue.srv.QueueProgram)(self._token, program_id)

        return program_id

    def run_program(self, program_id):
        if self._token is None:
            return False

        # run program
        print 'running program...'
        self._client('run_program', program_queue.srv.RunProgram)(self._token, program_id)

        return True

    def _client(self, name, type):
       rospy.wait_for_service(name, 3)
       return rospy.ServiceProxy(name, type)
