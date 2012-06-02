from copy import copy

from actions.ActionSequence import ActionSequence
from actions.DefaultAction import DefaultAction
from DoubleSpinBoxDelegate import DoubleSpinBoxDelegate
from IntegerSpinBoxDelegate import IntegerSpinBoxDelegate

import python_qt_binding.QtBindingHelper #@UnusedImport
from QtCore import QAbstractTableModel, QByteArray, QMimeData, QModelIndex, Qt, Signal

class PosesDataModel(QAbstractTableModel):

    actions_changed = Signal()
    duration_modified = Signal()

    _mime_type = 'application/x-slider-action'

    def __init__(self, editable=False):
        super(PosesDataModel, self).__init__()
        self._action_sequence = ActionSequence()
        self._editable = editable
        self._joint_columns = {}

        action = DefaultAction()
        self._add_joint(1, action, 'head_pan_joint', 'Hd Turn')
        self._add_joint(2, action, 'head_tilt_joint', 'Hd Nod')
        self._add_joint(3, action, 'torso_lift_joint', 'Torso', 2)
        self._add_joint(4, action, 'l_shoulder_pan_joint', 'Lp1')
        self._add_joint(5, action, 'l_shoulder_lift_joint', 'Ll2')
        self._add_joint(6, action, 'l_upper_arm_roll_joint', 'Lu3')
        self._add_joint(7, action, 'l_elbow_flex_joint', 'Le4')
        self._add_joint(8, action, 'l_forearm_roll_joint', 'Lf5')
        self._add_joint(9, action, 'l_wrist_flex_joint', 'Lw6')
        self._add_joint(10, action, 'l_wrist_roll_joint', 'Lr7')
        self._add_joint(11, action, 'l_gripper', 'LGrip', 3)
        self._add_joint(12, action, 'r_shoulder_pan_joint', 'Rp1')
        self._add_joint(13, action, 'r_shoulder_lift_joint', 'Rl2')
        self._add_joint(14, action, 'r_upper_arm_roll_joint', 'Ru3')
        self._add_joint(15, action, 'r_elbow_flex_joint', 'Re4')
        self._add_joint(16, action, 'r_forearm_roll_joint', 'Rf5')
        self._add_joint(17, action, 'r_wrist_flex_joint', 'Rw6')
        self._add_joint(18, action, 'r_wrist_roll_joint', 'Rr7')
        self._add_joint(19, action, 'r_gripper', 'RGrip', 3)

        # keep reference to delegates to prevent being garbaged
        self._delegates = []

    def _add_joint(self, column, action, label, header, precision=None):
        info = copy(action.get_joint_info(label))
        info['header'] = header
        info['decimals'] = precision
        self._joint_columns[column] = info

    def set_editable(self, editable):
        if self._editable != editable:
            self._editable = editable
            self.reset()

    def add_action(self, action, index = None):
        self._add_action(action, index)
        self.actions_changed.emit()
        self.duration_modified.emit()

    def _add_action(self, action, index = None):
        model_index = QModelIndex()
        if index is None or index == len(self._action_sequence.actions()):
            rows = len(self._action_sequence.actions())
            index = None
        else:
            assert(index >=0 and index < len(self._action_sequence.actions()))
            rows = index

        # insert at model-index of parent-item
        self.beginInsertRows(model_index, rows, rows)
        self._action_sequence.add_action(action, index)
        self.endInsertRows()

    def move_action(self, source_index, destination_index):
        assert(source_index >=0 and source_index < len(self._action_sequence.actions()))
        assert(destination_index >=0 and destination_index <= len(self._action_sequence.actions()))
        action = self._action_sequence.actions()[source_index]
        self._remove_action(source_index)
        if destination_index > source_index:
            destination_index -= 1
        self._add_action(action, destination_index)
        self.actions_changed.emit()

    def remove_action(self, index):
        self._remove_action(index)
        self.actions_changed.emit()
        self.duration_modified.emit()

    def _remove_action(self, index):
        assert(index >=0 and index < len(self._action_sequence.actions()))
        # delete at model-index of parent-item
        model_index = QModelIndex()
        self.beginRemoveRows(model_index, index, index)
        self._action_sequence.remove_action(index)
        self.endRemoveRows()

    def remove_all_actions(self):
        index = QModelIndex()
        rows = len(self._action_sequence.actions())
        self.beginRemoveRows(index, 0, rows - 1)
        self._action_sequence.remove_all_actions()
        self.endRemoveRows()
        self.actions_changed.emit()
        self.duration_modified.emit()

    def action_sequence(self):
        return self._action_sequence

    def rowCount(self, parent=None):
        return len(self._action_sequence.actions())

    def columnCount(self, parent=None):
        if self._editable:
            return 1 + len(self._joint_columns)
        else:
            return 1 + 1

    def data(self, index, role=None):
        if role is None:
            role = Qt.DisplayRole
        if index.row() >= 0 and index.row() < len(self._action_sequence.actions()):
            if role == Qt.DisplayRole:
                if index.column() == 0:
                    return '%.1f' % self._action_sequence.actions()[index.row()].get_duration()
                elif not self._editable and index.column() == 1:
                    return self._action_sequence.actions()[index.row()].to_string()
                elif self._editable and index.column() in self._joint_columns.keys():
                    joint_info = self._joint_columns[index.column()]
                    try:
                        value = self._action_sequence.actions()[index.row()].get_value(joint_info['label'])
                        if joint_info['decimals'] is not None:
                            value = round(value, joint_info['decimals'])
                        else:
                            value = int(round(value))
                        return value
                    except:
                        return ''
            if role == Qt.EditRole:
                if index.column() == 0:
                    return self._action_sequence.actions()[index.row()].get_duration()
                elif self._editable and index.column() in self._joint_columns.keys():
                    joint_info = self._joint_columns[index.column()]
                    try:
                        value = self._action_sequence.actions()[index.row()].get_value(joint_info['label'])
                        if joint_info['decimals'] is not None:
                            value = round(value, joint_info['decimals'])
                        else:
                            value = int(round(value))
                        return value
                    except:
                        return ''
        return None

    def setData(self, index, value, role):
        if role == Qt.EditRole:
            if index.column() == 0:
                value = float(value)
                self._action_sequence.actions()[index.row()].set_duration(value)
                self.duration_modified.emit()
                return True
            elif self._editable:
                value = float(value)
                try:
                    self._action_sequence.actions()[index.row()].update_value(self._joint_columns[index.column()]['label'], value)
                except:
                    return False
                return True
        return super(PosesDataModel, self).setData(index, value, role)

    def flags(self, index):
        f = super(PosesDataModel, self).flags(index)
        if index.isValid():
            if index.column() == 0:
                f |= Qt.ItemIsEditable
            elif self._editable and index.column() in self._joint_columns.keys():
                try:
                    # only cell which have real values can be edited
                    self._action_sequence.actions()[index.row()].get_value(self._joint_columns[index.column()]['label'])
                    f |= Qt.ItemIsEditable
                except:
                    pass
            f |= Qt.ItemIsDragEnabled
        f |= Qt.ItemIsDropEnabled
        return f

    def headerData(self, section, orientation, role=None):
        if role is None:
            role = Qt.DisplayRole
        if role == Qt.DisplayRole:
            if orientation == Qt.Horizontal:
                if section == 0:
                    return 'Duration'
                elif not self._editable and section == 1:
                    return 'Joints'
                elif self._editable and section in self._joint_columns.keys():
                    return self._joint_columns[section]['header']
            elif orientation == Qt.Vertical:
                return 'Pose %d' % (section + 1)
        return None

    def supportedDropActions(self):
        return Qt.CopyAction | Qt.MoveAction

    def mimeTypes(self):
        return [self._mime_type]

    def mimeData(self, indexes):
        #print 'mimeData()'
        row = None
        for index in indexes:
            if index.isValid():
                if row is None:
                    row = index.row()
                if row != index.row():
                    row = None
                    break

        mimeData = QMimeData()
        if row is not None:
            mimeData.setData(self._mime_type, QByteArray.number(row))
        return mimeData;

    def dropMimeData(self, data, action, row, column, parent):
        #print 'dropMimeData()'
        if action == Qt.MoveAction:
            before_row = None
            if row != -1:
                before_row = row
            elif parent.isValid():
                before_row = parent.row()
            else:
                before_row = self.rowCount()
            if data.hasFormat(self._mime_type):
                byte_array = data.data(self._mime_type)
                source_row, is_int = byte_array.toInt()
                if is_int and before_row != source_row + 1:
                    #print 'dropMimeData()', source_row, '->', before_row
                    self.move_action(source_row, before_row)
                    return True
        return super(PosesDataModel, self).dropMimeData(data, action, row, column, parent)

    def add_delegates(self, table_view):
        for i in self._joint_columns.keys():
            joint_info = self._joint_columns[i]
            if joint_info['decimals'] is not None:
                delegate = DoubleSpinBoxDelegate()
                delegate.setDecimals(joint_info['decimals'])
            else:
                delegate = IntegerSpinBoxDelegate()
            delegate.setMinimum(joint_info['min'])
            delegate.setMaximum(joint_info['max'])
            delegate.setSingleStep(joint_info['single_step'])
            table_view.setItemDelegateForColumn(i, delegate)
            self._delegates.append(delegate)
