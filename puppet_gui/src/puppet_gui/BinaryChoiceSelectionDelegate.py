from python_qt_binding import QtCore, QtGui
from IntegerSpinBoxDelegate import IntegerSpinBoxDelegate
from QtCore import Qt
from QtGui import QCheckBox, QItemDelegate


from python_qt_binding import QtCore, QtGui
from QtCore import Qt
from QtGui import QDoubleSpinBox

class BinaryChoiceSelectionDelegate(IntegerSpinBoxDelegate):

    def __init__(self, parent = None):
        super(BinaryChoiceSelectionDelegate, self).__init__(parent)
        self._textLabel = '<noLabel>'

    def setTextLabel(self, label):
        self._textLabel = label

    def createEditor(self, parent, label, index):
        editor = self._create_editor(parent)
        editor.installEventFilter(self)
        return editor

    def _create_editor(self, parent):
        return QCheckBox(self._textLabel, parent)
