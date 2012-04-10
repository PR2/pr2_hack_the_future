#!/usr/bin/env python

import os
import signal
from StringIO import StringIO
import sys
import tempfile

import roslib;
roslib.load_manifest('python_qt_binding')
roslib.load_manifest('rviz')
roslib.load_manifest('slider_gui')
import rospy;

#setattr(sys, 'SELECT_QT_BINDING', 'pyside')
from python_qt_binding.QtBindingHelper import loadUi
from QtCore import qFatal, QModelIndex, QObject, QRegExp, QSignalMapper, QTimer, Signal
from QtGui import QApplication, QDialog, QFileDialog, QIcon, QItemSelectionModel, QMainWindow, QMessageBox, QSplitter, QTableView, QVBoxLayout, QWidget
from DoubleSpinBoxDelegate import DoubleSpinBoxDelegate
from KontrolSubscriber import KontrolSubscriber
from PosesDataModel import PosesDataModel
from ProgramQueue import ProgramQueue
import rviz
from SimpleFormat import SimpleFormat

app = QApplication(sys.argv)

main_window = QMainWindow()
ui_file = os.path.join(os.path.dirname(os.path.realpath(__file__)), '..', 'src', 'slider_simple.ui')
loadUi(ui_file, main_window)

# set icons for tabs
icons = {
    0: 'square.png',
    1: 'triangle.png',
    2: 'circle.png',
    3: 'cross.png',
}
for index, filename in icons.iteritems():
    filename = os.path.realpath(os.path.join(os.path.dirname(__file__), '..', 'icons', filename))
    icon = QIcon(filename)
    if not icon.isNull():
        main_window.PoseList_tabWidget.setTabText(index, '')
        main_window.PoseList_tabWidget.setTabIcon(index, icon)

# hide design-only widgets
main_window.square_tableWidget.setVisible(False)

def sigint_handler(*args):
    print('\nsigint_handler()')
    main_window.close()
signal.signal(signal.SIGINT, sigint_handler)
# the timer enables triggering the sigint_handler
timer = QTimer()
timer.start(500)
timer.timeout.connect(lambda: None)

# replace placeholder with rviz visualization panel
index = main_window.robot_view_verticalLayout.indexOf(main_window.robot_view_widget)
stretch = main_window.robot_view_verticalLayout.stretch(index)
main_window.robot_view_verticalLayout.removeWidget(main_window.robot_view_widget)
robot_view = rviz.VisualizationPanel()
main_window.robot_view_verticalLayout.insertWidget(index, robot_view, stretch)
robot_view.setSizes([0])
config = tempfile.NamedTemporaryFile('w')
config.write("""Background\ ColorB=0
Background\ ColorG=0
Background\ ColorR=0
Camera\ Config=0.905202 5.83579 18.0278 0 -1.90735e-06 1.90735e-06
Camera\ Type=rviz::OrbitViewController
Fixed\ Frame=/base_link
Grid.Alpha=0.5
Grid.Cell\ Size=1
Grid.ColorB=0.5
Grid.ColorG=0.5
Grid.ColorR=0.5
Grid.Enabled=1
Grid.Line\ Style=0
Grid.Line\ Width=0.03
Grid.Normal\ Cell\ Count=0
Grid.OffsetX=0
Grid.OffsetY=0
Grid.OffsetZ=0
Grid.Plane=0
Grid.Plane\ Cell\ Count=10
Grid.Reference\ Frame=<Fixed Frame>
RobotModel.Alpha=1
RobotModel.Collision\ Enabled=0
RobotModel.Enabled=1
RobotModel.Robot\ Description=robot_description
RobotModel.TF\ Prefix=
RobotModel.Update\ Interval=0
RobotModel.Visual\ Enabled=1
Target\ Frame=<Fixed Frame>
[Display0]
ClassName=rviz::GridDisplay
Name=Grid
[Display1]
ClassName=rviz::RobotModelDisplay
Name=RobotModel
""")
config.flush()
robot_view.loadDisplayConfig(config.name)
config.close

views = []
view_mapper = QSignalMapper(main_window)
def set_view(index):
    robot_view.setViewString(views[index])
view_mapper.mapped.connect(set_view)
def view_selected(checked):
    if checked:
        view_mapper.map()
def add_view(button, view_str):
    view_mapper.setMapping(button, len(views))
    views.append(view_str)
    if button.isChecked():
        set_view(len(views) - 1)
    button.toggled.connect(view_mapper.map)
add_view(main_window.front_view_radioButton, "0.0402028 6.2758 2.24508 0.00208002 -0.0024735 0.753009")
add_view(main_window.side_view_radioButton, "-0.0747972 4.83578 2.81623 -0.0104112 -0.00416593 0.984444")
add_view(main_window.angled_view_radioButton, "0.475202 5.59079 2.81623 -0.0104112 -0.00416593 0.984444")

rospy.init_node('proto_simple', disable_signals=True)
try:
    use_sim_time = rospy.get_param('use_sim_time')
except KeyError:
    use_sim_time = False

kontrol_subscriber = KontrolSubscriber()

# pass signal across thread boundaries
class Foo(QObject):
    current_values_changed = Signal(str)
    current_duration_changed = Signal(float)
    _update_current_value_signal = Signal()
    def __init__(self):
        super(Foo, self).__init__()
        self._action_set = None
        self._update_current_value_signal.connect(self._update_current_value)
    def update_current_value(self):
        self._update_current_value_signal.emit()
    def _update_current_value(self):
        #print('update_current_value()')
        if self._action_set is not None:
            self._action_set.stop()
        self._action_set = kontrol_subscriber.get_action_set()
        self.current_duration_changed.emit(self._action_set.get_duration())
        value = self._action_set.to_string()
        self.current_values_changed.emit(value)
        for action in self._action_set._actions:
            action._duration = 0.1 if use_sim_time else 0.5
        self._action_set.execute()
foo = Foo()

kontrol_subscriber.axes_changed.connect(foo.update_current_value)
foo.current_values_changed.connect(main_window.lineEdit.setText)
foo.current_duration_changed.connect(main_window.duration_doubleSpinBox.setValue)

def get_tab_widget(index):
    return main_window.PoseList_tabWidget.widget(index)

def get_table_view(index):
    tab_widget = get_tab_widget(index)
    return tab_widget.findChildren(QTableView, QRegExp('.*_tableView'))[0]

def get_current_tab_index():
    return main_window.PoseList_tabWidget.currentIndex()

def test_clicked(index):
    model = get_current_model()
    print 'execute %d' % index.row()
    model.action_sequence().actions()[index.row()].execute()

# create models for each table view in the tabs
models = []
for i in range(main_window.PoseList_tabWidget.count()):
    table_view = get_table_view(i)
    model = PosesDataModel()
    table_view.setModel(model)
    table_view.resizeColumnsToContents()
    delegate = DoubleSpinBoxDelegate()
    delegate.setMinimum(main_window.duration_doubleSpinBox.minimum())
    delegate.setMaximum(main_window.duration_doubleSpinBox.maximum())
    delegate.setSuffix(main_window.duration_doubleSpinBox.suffix())
    delegate.setSingleStep(main_window.duration_doubleSpinBox.singleStep())
    table_view.setItemDelegateForColumn(0, delegate)
    table_view.doubleClicked.connect(test_clicked)
    models.append(model)

def get_current_model():
    index = get_current_tab_index()
    return models[index]


def append_current():
    model = get_current_model()

    action_set = kontrol_subscriber.get_action_set()
    model.add_action(action_set)

    table_view = get_table_view(get_current_tab_index())
    rows = len(model.action_sequence().actions())
    table_view.resizeColumnsToContents()
    table_view.selectRow(rows - 1)

main_window.append_pushButton.clicked.connect(append_current)


def insert_current_before_selected():
    row = get_selected_row()
    if row is None:
        append_current()
        return

    model = get_current_model()

    action_set = kontrol_subscriber.get_action_set()
    model.add_action(action_set, row)

    table_view = get_table_view(get_current_tab_index())
    table_view.resizeColumnsToContents()
    table_view.selectRow(row + 1)

main_window.insert_before_pushButton.clicked.connect(insert_current_before_selected)


def delete_selected():
    row = get_selected_row()
    if row is None:
        return

    model = get_current_model()
    model.remove_action(row)

    table_view = get_table_view(get_current_tab_index())
    rows = len(model.action_sequence().actions())
    if row >= rows:
        row = row - 1
    table_view.resizeColumnsToContents()
    table_view.selectRow(row)

main_window.delete_selected_pushButton.clicked.connect(delete_selected)


def clone_selected():
    row = get_selected_row()
    if row is None:
        return

    model = get_current_model()
    action = model.action_sequence().actions()[row]
    clone = action.deepcopy()
    model.add_action(clone, row + 1)

main_window.clone_selected_pushButton.clicked.connect(clone_selected)


def get_row_count():
    model = get_current_model()
    return model.rowCount()
def get_selected_row():
    table_view = get_table_view(get_current_tab_index())
    selection_model = table_view.selectionModel()
    if selection_model.hasSelection():
        row = selection_model.selectedRows()[0].row()
        #print 'get_selected_row() %d' % row
        return row
    #print 'get_selected_row() None'
    return None
def test_selected():
    row = get_selected_row()
    if row is not None:
        model = get_current_model()
        print 'execute %d' % row
        model.action_sequence().actions()[row].execute()

main_window.test_selected_pushButton.clicked.connect(test_selected)


def set_selected_row(row):
    #print 'set_selected_row(%d)' % row
    rows = get_row_count()
    if rows == 0:
        return
    if row < 0:
        row = 0
    if row >= rows:
        row = rows - 1
    table_view = get_table_view(get_current_tab_index())
    #print 'set_selected_row() row %d' % row
    table_view.selectRow(row)

class RelaySignalInt(QObject):
    relay_signal = Signal(int)
    def __init__(self):
        super(RelaySignalInt, self).__init__()
    def emit(self, row):
        self.relay_signal.emit(row)

set_selected_row_signal = RelaySignalInt()
set_selected_row_signal.relay_signal.connect(set_selected_row)

running_sequence = False

def finished_executing_current_sequence():
    global running_sequence
    print 'finished_executing_current_sequence()'
    model = get_current_model()
    action_sequence = model.action_sequence()
    action_sequence.executing_action_signal.disconnect(set_selected_row_signal.emit)
    action_sequence.execute_sequence_finished_signal.disconnect(finished_executing_current_sequence)
    running_sequence = False

def execute_current_sequence():
    global running_sequence
    if running_sequence:
        print 'execute_current_sequence() skipped'
        return
    print 'execute_current_sequence()'
    running_sequence = True
    model = get_current_model()
    action_sequence = model.action_sequence()
    action_sequence.executing_action_signal.connect(set_selected_row_signal.emit)
    action_sequence.execute_sequence_finished_signal.connect(finished_executing_current_sequence)
    action_sequence.execute_all()

def stop_sequence():
    global running_sequence
    if not running_sequence:
        print 'stop_sequence() skipped - not running'
        return
    print 'stop_sequence()'
    model = get_current_model()
    action_sequence = model.action_sequence()
    action_sequence.stop()
    action_sequence.executing_action_signal.disconnect(set_selected_row_signal.emit)
    action_sequence.execute_sequence_finished_signal.disconnect(finished_executing_current_sequence)
    running_sequence = False

def set_tab(index):
    main_window.PoseList_tabWidget.setCurrentIndex(index)


select_row_signal = RelaySignalInt()
select_row_signal.relay_signal.connect(set_selected_row)

def check_buttons():
    triggered_buttons = kontrol_subscriber.get_triggered_buttons()

    if KontrolSubscriber.previous_button in triggered_buttons:
        if get_selected_row() is not None:
            select_row_signal.emit(get_selected_row() - 1)
        else:
            select_row_signal.emit(0)
    elif KontrolSubscriber.play_button in triggered_buttons:
        test_selected()
    elif KontrolSubscriber.next_button in triggered_buttons:
        if get_selected_row() is not None:
            select_row_signal.emit(get_selected_row() + 1)
        else:
            select_row_signal.emit(get_row_count() - 1)
    
    elif KontrolSubscriber.repeat_button in triggered_buttons:
        execute_current_sequence()
    elif KontrolSubscriber.stop_button in triggered_buttons:
        stop_sequence()
    elif KontrolSubscriber.record_button in triggered_buttons:
        append_current()

    elif KontrolSubscriber.top1_button in triggered_buttons:
        set_tab(0)
    elif KontrolSubscriber.bottom1_button in triggered_buttons:
        set_tab(3)
    elif KontrolSubscriber.top2_button in triggered_buttons:
        set_tab(1)
    elif KontrolSubscriber.bottom2_button in triggered_buttons:
        set_tab(2)

class RelaySignal(QObject):
    relay_signal = Signal()
    def __init__(self):
        super(RelaySignal, self).__init__()
    def emit(self):
        self.relay_signal.emit()

check_buttons_signal = RelaySignal()
check_buttons_signal.relay_signal.connect(check_buttons)
kontrol_subscriber.buttons_changed.connect(check_buttons_signal.emit)


def load_from_file():
    file_name, _ = QFileDialog.getOpenFileName(main_window, main_window.tr('Load program from file'), None, main_window.tr('Puppet Talk (*.pt)'))
    if file_name is None or file_name == '':
        return

    print 'load_from_file', file_name
    handle = open(file_name, 'rb')
    storage = SimpleFormat(handle)

    count = storage.deserialize_data()
    tabs = main_window.PoseList_tabWidget.count()
    for index in range(tabs):
        model = models[index]
        if index < count:
            model.action_sequence().deserialize(storage)
            model.reset()
        else:
            model.remove_all_actions()
    handle.close()

main_window.actionOpen.triggered.connect(load_from_file)


def serialize(storage):
    count = main_window.PoseList_tabWidget.count()
    storage.serialize_data(count)
    for index in range(count):
        model = models[index]
        model.action_sequence().serialize(storage)

def save_to_file():
    file_name, _ = QFileDialog.getSaveFileName(main_window, main_window.tr('Save program to file'), 'example.pt', main_window.tr('Puppet Talk (*.pt)'))
    if file_name is None or file_name == '':
        return

    print 'save_to_file', file_name
    handle = open(file_name, 'wb')
    storage = SimpleFormat(handle)
    serialize(storage)
    handle.close()

main_window.actionSave_As.triggered.connect(save_to_file)


def clear_all():
    print 'clear_all'
    for index in range(main_window.PoseList_tabWidget.count()):
        model = models[index]
        model.remove_all_actions()

main_window.actionClear_All.triggered.connect(clear_all)

main_window.actionTest_Program.triggered.connect(execute_current_sequence)


def queue_program():
    dialog = QDialog()
    ui_file = os.path.join(os.path.dirname(os.path.realpath(__file__)), '..', 'src', 'queue_dialog.ui')
    loadUi(ui_file, dialog)
    rc = dialog.exec_()
    if rc == QDialog.Rejected:
        return

    queue = ProgramQueue(dialog.username_lineEdit.text(), dialog.password_lineEdit.text())
    rc = queue.login()
    if not rc:
        QMessageBox.critical(main_window, main_window.tr('Login failed'), main_window.tr('Could not log in to the program queue.'))
        return

    output = StringIO()
    storage = SimpleFormat(output)
    serialize(storage)
    program_data = output.getvalue()
    output.close()

    id = queue.upload_program(program_data, dialog.label_lineEdit.text())
    if not id:
        QMessageBox.critical(main_window, main_window.tr('Upload failed'), main_window.tr('Could not upload program to queue.'))
        return
    queue.logout()
    QMessageBox.information(main_window, main_window.tr('Uploaded program'), main_window.tr('Uploaded program (%d) successfully.') % id)

main_window.actionQueue_Program.triggered.connect(queue_program)


main_window.actionExit.triggered.connect(main_window.close)

#main_window.show()
main_window.showMaximized()

sys.exit(app.exec_())
