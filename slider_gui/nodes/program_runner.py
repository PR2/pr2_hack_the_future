#!/usr/bin/env python

from StringIO import StringIO
from threading import Event

import roslib;
roslib.load_manifest('slider_gui')
import rospy

from actions.ActionSequence import ActionSequence
from Ps3Subscriber import Ps3Subscriber
from SimpleFormat import SimpleFormat
from slider_gui.srv import *

class Runner:
    def __init__(self):
        self._event = None
        self._sequences = None
        self._running_sequence = None

        self._ps3_subscriber = Ps3Subscriber()
        self._ps3_subscriber.buttons_changed.connect(self._check_ps3_buttons)

        rospy.Service('run_slider_program', RunProgram, self._handle_run_program)
        rospy.loginfo('Runner ready')

    def _handle_run_program(self, req):
        print 'Running program...'

        input = StringIO(req.code)
        storage = SimpleFormat(input)
        count = storage.deserialize_data()
        assert(count > 0)
        self._sequences = []
        for index in range(count):
            sequence = ActionSequence()
            sequence.deserialize(storage)
            print 'Loaded sequence %d with %d poses' % (index + 1, len(sequence.actions()))
            sequence.executing_action_signal.connect(self._progress)
            sequence.execute_sequence_finished_signal.connect(self._finished)
            self._sequences.append(sequence)

        self._event = Event()
        self._event.wait()

        self._stop_current_sequence()

        for sequence in self._sequences:
            sequence.executing_action_signal.disconnect(self._progress)
            sequence.execute_sequence_finished_signal.disconnect(self._finished)
        self._sequences = None

        print 'Running program finished'
        return RunProgramResponse()

    def _check_ps3_buttons(self):
        triggered_buttons = self._ps3_subscriber.get_triggered_buttons()

        if Ps3Subscriber.select_button in triggered_buttons:
            self._event.set()
        elif Ps3Subscriber.start_button in triggered_buttons:
            #default_pose()
            pass

        elif Ps3Subscriber.square_button in triggered_buttons:
            self._execute_sequence(0)
        elif Ps3Subscriber.triangle_button in triggered_buttons:
            self._execute_sequence(1)
        elif Ps3Subscriber.circle_button in triggered_buttons:
            self._execute_sequence(2)
        elif Ps3Subscriber.cross_button in triggered_buttons:
            self._execute_sequence(3)

    def _execute_sequence(self, index):
        self._stop_current_sequence()
        if index in range(len(self._sequences)):
            print 'Execute sequence %d' % (index + 1)
            self._running_sequence = index
            sequence = self._sequences[index]
            sequence.execute_all()

    def _progress(self, index):
        print 'Step %d done' % (index + 1)

    def _finished(self):
        self._running_sequence = None

    def _stop_current_sequence(self):
        if self._running_sequence is not None:
            print 'Stop sequence %d' % (self._running_sequence + 1)
            sequence = self._sequences[self._running_sequence]
            sequence.stop()
            self._running_sequence = None


if __name__ == '__main__':
    rospy.init_node('program_runner')
    runner = Runner()
    rospy.spin()
