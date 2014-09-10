from IntegerSpinBoxDelegate import IntegerSpinBoxDelegate

import python_qt_binding.QtBindingHelper #@UnusedImport
from QtCore import Qt
from QtGui import QDoubleSpinBox

class DoubleSpinBoxDelegate(IntegerSpinBoxDelegate):

    def __init__(self, parent = None):
        super(DoubleSpinBoxDelegate, self).__init__(parent)
        self._decimals = None

    def setDecimals(self, decimals):
        self._decimals = decimals

    def createEditor(self, parent, option, index):
        editor = super(DoubleSpinBoxDelegate, self).createEditor(parent, option, index)
        if self._decimals is not None:
            editor.setDecimals(self._decimals)
        return editor

    def _create_editor(self, parent):
        return QDoubleSpinBox(parent)
