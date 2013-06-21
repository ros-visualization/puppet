from python_qt_binding import QtCore
from QtCore import Qt, QSize, QEvent
from QtGui import QCheckBox, QStyledItemDelegate

from speechActionEditor import SpeechActionEditor;

class SpeechEditDelegate(QStyledItemDelegate):

    def __init__(self, parent = None):
        super(SpeechEditDelegate, self).__init__(parent)
        self.parent = parent;
        self._textLabel = '<noLabel>'
        self.editor = None
        

    def setTextLabel(self, label):
        self._textLabel = label

    def createEditor(self, parent, label, index):
        model = index.model()
        data = model.data(index)
        if not model.editable() or ((data[0] is None) and (data[1] is None)):
            return None
        
        self.editor = self._create_editor(parent)
        self.editor.installEventFilter(self)
        return self.editor

    def eventFilter(self, obj, event):
        type = event.type()
        if type == QEvent.Leave:
            self.endEdit()
        return False
    
    def editorEvent(self, event, model, option, index):
        '''
        Called by system when a speech editor is started. We
        save the parameters, so that we can later ensure that
        the model is updated from the editor data.
        @param event:
        @type event:
        @param model:
        @type model:
        @param option:
        @type option:
        @param index:
        @type index:
        '''
        self.currModel  = model
        self.currIndex  = index
        # Give others a chance to handle this event
        return False
    
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
        
    def endEdit(self):
        '''
        Make sure data is put into editor even when
        editing a cell is terminated by clicking on 
        the 'Standard' radio button:
        '''
        self.setModelData(self.editor, self.currModel, self.currIndex)

#    def updateEditorGeometry(self, editor, option, index):
#        editor.setGeometry(editor.sizeHint())

    def sizeHint (self, option, index):
        '''
        @param option: any style options
        @type option: QStyleOptionViewItem 
        @param index: index into model
        @type index: QModelIndex
        '''
        #cellStr = index.model().data(index)
        if index.model().editable():
            # If editable, need vertical
            # space for the text edit field:
            size = QSize(400, 400)
        else:
            size = QSize(400, 100)
        return size
        #***************8
        #return QSize(100,300)
        #return self._create_editor(self.parent).sizeHint();
        #***************8
    
