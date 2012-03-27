from actions.ActionSequence import ActionSequence

import python_qt_binding.QtBindingHelper #@UnusedImport
from QtCore import QAbstractItemModel, QModelIndex, Qt

class PosesDataModel(QAbstractItemModel):

    def __init__(self):
        super(PosesDataModel, self).__init__()
        self._action_sequence = ActionSequence()

    def add_action(self, action):
        index = QModelIndex()
        rows = len(self._action_sequence.actions())

        # insert at model-index of parent-item
        self.beginInsertRows(index, rows, rows)
        self._action_sequence.add_action(action)
        self.endInsertRows()

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
        return 1

    def data(self, index, role=None):
        if role is None:
            role = Qt.DisplayRole
        if index.row() >= 0 and index.row() < len(self._action_sequence.actions()) and role == Qt.DisplayRole:
            return self._action_sequence.actions()[index.row()].to_string()
        return None

    def headerData(self, section, orientation, role=None):
        if role is None:
            role = Qt.DisplayRole
        if role == Qt.DisplayRole:
            if orientation == Qt.Horizontal:
                assert(section == 0)
                return 'Pose sequence'
            elif orientation == Qt.Vertical:
                return 'Pose %d' % (section + 1)
        return None
