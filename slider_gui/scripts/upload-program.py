#!/usr/bin/env python

from optparse import OptionParser
import os
import sys

import roslib;
roslib.load_manifest('program_queue')
roslib.load_manifest('slider_gui')
import rospy;

from slider_gui.srv import *

import program_queue.msg
import program_queue.srv

#from KontrolSubscriber import KontrolSubscriber
#from SimpleFormat import SimpleFormat

def client(name, type):
   rospy.wait_for_service(name)
   return rospy.ServiceProxy(name, type)

def upload_program(program_data, label, username, password):
    print 'upload_program(..., %s, %s, %s)' % (label, username, password)

    # log in
    print 'logging in...'
    user = client('login', program_queue.srv.Login)(username, password)
    token = user.token
    if token == 0:
        print 'Failed to log in'
        return 1

    # create empty program
    print 'creating program...'
    program = client('create_program', program_queue.srv.CreateProgram)(token)
    program_id = program.id

    # upload program data
    print 'uploading program...'
    program_info = program_queue.msg.ProgramInfo(program_id, label, program_queue.msg.ProgramInfo.SLIDER, username)
    program = program_queue.msg.Program(program_info, program_data)
    client('update_program', program_queue.srv.UpdateProgram)(token, program)

    # run program
    print 'running program...'
    client('run_program', program_queue.srv.RunProgram)(token, program_id)

    # log out
    print 'logging out...'
    client('logout', program_queue.srv.Logout)(token)

    return 0


if __name__ == '__main__':
    parser = OptionParser('usage: %prog [options] filename')

    parser.add_option('-u', '--username', dest='username', default='admin', type='str', metavar='USERNAME',
                      help='The username for the service calls')
    parser.add_option('-p', '--password', dest='password', default='admin', type='str', metavar='PASSWORD',
                      help='The password for the service calls')
    parser.add_option('-l', '--label', dest='label', default='program', type='str', metavar='LABEL',
                      help='The label for the uploaded program')

    options, args = parser.parse_args(sys.argv[1:])
    if len(args) != 1:
        parser.parse_args(['--help'])
        # calling --help will exit
    filename = args[0]

    if not os.path.isfile(filename):
        print 'could not find file %s' % filename
        sys.exit(-1)

    program_data = open(filename, 'rb').read()
    sys.exit(upload_program(program_data, options.label, options.username, options.password))
