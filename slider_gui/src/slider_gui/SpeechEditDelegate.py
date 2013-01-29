#import python_qt_binding.QtBindingHelper #@UnusedImport
from python_qt_binding import QtCore
from QtCore import Qt, QSize
from QtGui import QCheckBox, QStyledItemDelegate

from speechActionEditor import SpeechActionEditor;

class SpeechEditDelegate(QStyledItemDelegate):

    def __init__(self, parent = None):
        super(SpeechEditDelegate, self).__init__(parent)
        self.parent = parent;
        self._textLabel = '<noLabel>'

    def setTextLabel(self, label):
        self._textLabel = label

    def createEditor(self, parent, label, index):
        editor = self._create_editor(parent)
        editor.installEventFilter(self)
        return editor

    def _create_editor(self, parent):
        #return QCheckBox(self._textLabel, parent)
        return SpeechActionEditor(parent);

    def setEditorData(self, editor, index):
        # Get tuple: waitForSpeechDone,TextToSpeak:
        value = index.model().data(index, Qt.EditRole)
        editor.setValue(value)

    def setModelData(self, editor, model, index):
        value = editor.value()
        model.setData(index, value, Qt.EditRole)
        

    def updateEditorGeometry(self, editor, option, index):
        editor.setGeometry(option.rect)

    def sizeHint (self, option, index):
        '''
        @param option: any style options
        @type option: QStyleOptionViewItem 
        @param index: index into model
        @type index: QModelIndex
        '''
        #height = max(self._textLabel.height, self.
        #***************8
        return QSize(100,300)
        #return self._create_editor(self.parent).sizeHint();
        #***************8
    
