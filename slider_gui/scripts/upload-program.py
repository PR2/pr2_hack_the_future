#!/usr/bin/env python

from optparse import OptionParser
import os
import sys

import roslib;
roslib.load_manifest('slider_gui')

from ProgramQueue import ProgramQueue

if __name__ == '__main__':
    parser = OptionParser('usage: %prog [options] filename')

    parser.add_option('-u', '--username', dest='username', default='admin', type='str', metavar='USERNAME',
                      help='The username for the service calls')
    parser.add_option('-p', '--password', dest='password', default='admin', type='str', metavar='PASSWORD',
                      help='The password for the service calls')
    parser.add_option('-l', '--label', dest='label', default='program', type='str', metavar='LABEL',
                      help='The label for the uploaded program')
    parser.add_option('-r', '--run', dest='run', default=False, action="store_true",
                      help='Run the program after uploading it')

    options, args = parser.parse_args(sys.argv[1:])
    if len(args) != 1:
        parser.parse_args(['--help'])
        # calling --help will exit
    filename = args[0]

    if not os.path.isfile(filename):
        print 'could not find file %s' % filename
        sys.exit(-1)

    program_data = open(filename, 'rb').read()

    queue = ProgramQueue(options.username, options.password)

    try:
        rc = queue.login()
    except:
        rc = False
    if not rc:
        sys.exit(1)

    try:
        id = queue.upload_program(program_data, options.label)
    except:
        id = False
    if not id:
        sys.exit(2)

    if options.run:
        try:
            rc = queue.run_program(id)
        except:
            rc = False
        if not rc:
            sys.exit(3)

    try:
        rc = queue.logout()
    except:
        rc = False
    if not rc:
        sys.exit(4)

    sys.exit(0)
