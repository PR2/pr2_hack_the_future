import python_qt_binding.QtBindingHelper #@UnusedImport
from QtCore import Qt
from QtGui import QDoubleSpinBox, QItemDelegate

class DoubleSpinBoxDelegate(QItemDelegate):

    def __init__(self, parent = None):
        super(DoubleSpinBoxDelegate, self).__init__(parent)
        self._minimum = None
        self._maximum = None
        self._suffix = None
        self._single_step = None

    def setMinimum(self, minimum):
        self._minimum = minimum

    def setMaximum(self, maximum):
        self._maximum = maximum

    def setSuffix(self, suffix):
        self._suffix = suffix

    def setSingleStep(self, single_step):
        self._single_step = single_step

    def createEditor(self, parent, option, index):
        editor = QDoubleSpinBox(parent)
        if self._minimum is not None:
            editor.setMinimum(self._minimum)
        if self._maximum is not None:
            editor.setMaximum(self._maximum)
        if self._suffix is not None:
            editor.setSuffix(self._suffix)
        if self._single_step is not None:
            editor.setSingleStep(self._single_step)
        return editor

    def setEditorData(self, editor, index):
        value = index.model().data(index, Qt.EditRole)
        editor.setValue(value)

    def setModelData(self, editor, model, index):
     editor.interpretText()
     value = editor.value()
     model.setData(index, value, Qt.EditRole)

    def updateEditorGeometry(self, editor, option, index):
        editor.setGeometry(option.rect)
