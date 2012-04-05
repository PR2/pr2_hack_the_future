from actions.ActionSequence import ActionSequence

import python_qt_binding.QtBindingHelper #@UnusedImport
from QtCore import QAbstractTableModel, QByteArray, QMimeData, QModelIndex, Qt

class PosesDataModel(QAbstractTableModel):

    _mime_type = 'application/x-slider-action'

    def __init__(self):
        super(PosesDataModel, self).__init__()
        self._action_sequence = ActionSequence()

    def add_action(self, action, index = None):
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
        self.remove_action(source_index)
        if destination_index > source_index:
            destination_index -= 1
        self.add_action(action, destination_index)

    def remove_action(self, index):
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

    def action_sequence(self):
        return self._action_sequence

    def rowCount(self, parent=None):
        return len(self._action_sequence.actions())

    def columnCount(self, parent=None):
        return 2

    def data(self, index, role=None):
        if role is None:
            role = Qt.DisplayRole
        if index.row() >= 0 and index.row() < len(self._action_sequence.actions()):
            if role == Qt.DisplayRole:
                if index.column() == 0:
                    return '%.1f' % self._action_sequence.actions()[index.row()].get_duration()
                elif index.column() == 1:
                    return self._action_sequence.actions()[index.row()].to_string()
            if role == Qt.EditRole and index.column() == 0:
                return self._action_sequence.actions()[index.row()].get_duration()
        return None

    def setData(self, index, value, role):
        if role == Qt.EditRole and index.column() == 0:
            value = float(value)
            self._action_sequence.actions()[index.row()].set_duration(value)
            return True
        return super(PosesDataModel, self).setData(index, value, role)

    def flags(self, index):
        f = super(PosesDataModel, self).flags(index)
        if index.isValid():
            if index.column() == 0:
                f |= Qt.ItemIsEditable
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
                elif section == 1:
                    return 'Pose sequence'
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
