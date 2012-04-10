#!/usr/bin/env python

from StringIO import StringIO
from threading import Event

import roslib;
roslib.load_manifest('slider_gui')
import rospy

from actions.ActionSequence import ActionSequence
from SimpleFormat import SimpleFormat
from slider_gui.srv import *

class Runner:
    def __init__(self):
        self._event = None

        rospy.Service('run_slider_program', RunProgram, self._handle_run_program)
        rospy.loginfo('Runner ready')

    def _handle_run_program(self, req):
        print 'Running program...'

        input = StringIO(req.code)
        storage = SimpleFormat(input)
        count = storage.deserialize_data()
        assert(count > 0)
        sequence = ActionSequence()
        sequence.deserialize(storage)
        print 'Loaded first sequence with %d poses' % len(sequence.actions())

        self._event = Event()
        sequence.executing_action_signal.connect(self._progress)
        sequence.execute_sequence_finished_signal.connect(self._finished)
        sequence.execute_all()
        self._event.wait()

        sequence.executing_action_signal.disconnect(self._progress)
        sequence.execute_sequence_finished_signal.disconnect(self._finished)

        print 'Running program finished'
        return RunProgramResponse()

    def _progress(self, index):
        print 'Step %d done' % (index + 1)

    def _finished(self):
        self._event.set()


if __name__ == '__main__':
    rospy.init_node('program_runner')
    runner = Runner()
    rospy.spin()
