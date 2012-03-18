#!/usr/bin/env python

import os
import sys

import roslib;
roslib.load_manifest('python_qt_binding')
roslib.load_manifest('slider_gui')
import rospy;

from python_qt_binding.QtBindingHelper import loadUi
from QtCore import qFatal, QObject, QTimer, Signal
from QtGui import QApplication, QMainWindow
from KontrolSubscriber import KontrolSubscriber

app = QApplication(sys.argv)

main_window = QMainWindow()
ui_file = os.path.join(os.path.dirname(os.path.realpath(__file__)), '..', 'src', 'sharon_simple.ui')
loadUi(ui_file, main_window)

rospy.init_node('proto_simple', disable_signals=True)

kontrol_subscriber = KontrolSubscriber()

# pass signal across thread boundaries
class Foo(QObject):
    current_value_changed = Signal(str)
    def update_current_value(self):
        #print('update_current_value()')
        action_set = kontrol_subscriber.get_action_set()
        action_set.execute()
        value = action_set.to_string()
        self.current_value_changed.emit(value)
foo = Foo()

kontrol_subscriber.new_message.connect(foo.update_current_value)
foo.current_value_changed.connect(main_window.lineEdit.setText)

main_window.show()

sys.exit(app.exec_())
