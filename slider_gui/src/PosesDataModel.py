from actions.ActionSequence import ActionSequence

import python_qt_binding.QtBindingHelper #@UnusedImport
from QtCore import QAbstractItemModel, QModelIndex, Qt

class PosesDataModel(QAbstractItemModel):

    def __init__(self):
        super(PosesDataModel, self).__init__()
        self._action_sequence = ActionSequence()

    def add_action(self, action, index = None):
        model_index = QModelIndex()
        if index is None:
            rows = len(self._action_sequence.actions())
        else:
            assert(index >=0 and index < len(self._action_sequence.actions()))
            rows = index

        # insert at model-index of parent-item
        self.beginInsertRows(model_index, rows, rows)
        self._action_sequence.add_action(action, index)
        self.endInsertRows()

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

    def index(self, row, column, parent=None):
        if parent is None:
            parent = QModelIndex()
        if not self.hasIndex(row, column, parent):
            return QModelIndex();
        if row >= 0 and row < len(self._action_sequence.actions()):
            return self.createIndex(row, column, self._action_sequence.actions()[row])
        else:
            return QModelIndex()

    def parent(self, index):
        return QModelIndex()

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
        if index.column() == 0:
            f |= Qt.ItemIsEditable
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
