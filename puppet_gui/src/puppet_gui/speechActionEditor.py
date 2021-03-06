from python_qt_binding import QtCore, QtGui
from QtCore import Qt, QSize, QRect
from QtGui import QApplication, QWidget, QItemDelegate, QPlainTextEdit, QDialog, QFrame, QHBoxLayout, QVBoxLayout, QLabel, QMainWindow, QCheckBox

import os
import sys

class SpeechActionEditor(QWidget):
    
    SPEECH_EDIT_FLD_WIDTH  = 400
    SPEECH_EDIT_FLD_HEIGHT = 300
    
    def __init__(self, parent=None):
        super(SpeechActionEditor, self).__init__(parent);
        #ui_file = os.path.join(os.path.dirname(os.path.realpath(__file__)), '..', 'src', 'speech_data_widget.ui')
        #editWidget = loadUi(ui_file, parent)
        
        # Must save the QFrame that holds the checkbox and text box,
        # so that the underlying C objects aren't GCed:
        self.editWidget = self.createWidget(parent);
    
    def createWidget(self, parent=None):
        hbox = QHBoxLayout();
        self.waitYesNoCheckbox = QCheckBox('Await speech done');
        hbox.addWidget(self.waitYesNoCheckbox);
        hbox.addWidget(QLabel('Text'));
        self.speechTextField = QPlainTextEdit();
        self.speechTextField.setMaximumWidth(SpeechActionEditor.SPEECH_EDIT_FLD_WIDTH);
        hbox.addWidget(self.speechTextField);
        self.setLayout(hbox);
        self.setMaximumHeight(SpeechActionEditor.SPEECH_EDIT_FLD_HEIGHT + 10);
        self.setMaximumWidth(SpeechActionEditor.SPEECH_EDIT_FLD_WIDTH + self.waitYesNoCheckbox.width());
        return self
        
    def setValue(self, blockBoolAndTextTuple):
        waitForSpeechDone, text = blockBoolAndTextTuple 
        if waitForSpeechDone:
            self.waitYesNoCheckbox.setCheckState(Qt.Checked)
        else:
            self.waitYesNoCheckbox.setCheckState(Qt.Unchecked)
        self.speechTextField.setPlainText(text);
        
    def value(self):
        '''
        Returns a tuple containing whether motions are to be suspended
        till speech is done, and the text to be spoken.
        @return: block instruction and text to speak
        @rtype: (boolean, string)
        '''
        text = self.speechTextField.toPlainText();
        waitForSpeechDone = self.waitYesNoCheckbox.checkState() == Qt.Checked;
        return (waitForSpeechDone, text);

    def sizeHint (self):
        '''
        @param option: any style options
        @type option: QStyleOptionViewItem 
        @param index: index into model
        @type index: QModelIndex
        '''

        # Need to allow room for the checkbox, checkbox label,
        # and a text field:
        (waitForSpeechDone, utterance) = self.value()
        #width = self.waitYesNoCheckbox.width + self.speechTextField.width
        width = self.maximumWidth()
        #height = self.waitYesNoCheckbox.height + self.speechTextField.height
        height = self.maximumHeight()
    
        return QSize(width, height)
        
if __name__=='__main__':
    app = QApplication(sys.argv)
    main_window = QMainWindow()
    ed = SpeechActionEditor()
    ed.show()
    sys.exit(app.exec_())

