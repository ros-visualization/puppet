#!/bin/env python

import roslib;
roslib.load_manifest('puppet_gui');
roslib.load_manifest('python_qt_binding');

import os;
import sys;
import string;

from python_qt_binding import QtBindingHelper;
from python_qt_binding.QtBindingHelper import loadUi;
from QtCore import Signal, Slot;
from QtGui import QApplication, QMainWindow, QFileDialog, QMessageBox, QErrorMessage, QFileDialog;

QT_CREATOR_UI_FILE_ROOT = os.path.join(os.path.dirname(os.path.realpath(__file__)), "../QtCreatorFiles");

class PythonScriptDialog(QMainWindow): 
    '''
    Class translates Puppet script poses to corresponding Python script statement
    sequences. The result is displayed in a Qt QMainWindow that the Puppet main
    application can expose and hide as needed. The window contains four buttons for: 
      - Copying all of the Python statements to the X cut buffer.
      - Saving the statements to a file.
      - Clearing the window, and
      - Closing the window.
    The class is a singleton.
    '''
    
    # Singleton instance, obtainable via this
    # class var.
    ui = None;
                
    def __init__(self, parent=None):
        # Enforce singleton op for this class. 
        # We could simply return, if we detect
        # a second instantiation here, but it seems
        # better to clarify the correct operation: 
        if PythonScriptDialog.ui is not None:
            raise RuntimeError("Can only have single instance of PythonScriptDialog. Use getInstance() factory method.");
        
        super(PythonScriptDialog, self).__init__(parent);
        self.dialogService = DialogService();
        self.recentDirSavedTo = os.getenv("HOME", "/tmp");
            
        self.initUI();
        # Move this window to the center of the screen,
        # else it always pops up far away:
        self.move(QApplication.desktop().screen().rect().center() - self.rect().center())

        # Dict that maps Puppet function names to methods in this class
        # that produce respective translation to Python script:
        self.TRANSLATION_METHOD_MAP = {
                                	   'head' : self.headToPython,
                                	   'rarm' : self.rarmToPython,
                                	   'larm' : self.larmToPython,
                                	   'lgrip': self.lgripToPython,
                                	   'rgrip': self.rgripToPython,
                                	   'speak': self.speakToPython,
                                	   'look_at_face' : self.lookAtFaceToPython,
                                	   'torso': self.torsoToPython
                                	  }

    def initUI(self):
        '''
        Create convenient button names, and connect buttons to actions.
        '''
        
        self.setWindowTitle("Python Script");
        
        # Load QtCreator's XML UI files:
        # Make QtCreator generated UIs children if this instance:
        self.loadUIs();
        self.copyAllButton = self.scriptDialog.copyAllButton;
        self.clearAllButton =  self.scriptDialog.clearAllButton;
        self.closeButton = self.scriptDialog.closeButton;
        self.saveAsButton = self.scriptDialog.saveAsButton;
        
        self.copyAllButton.clicked.connect(self.copyAllAction);
        self.clearAllButton.clicked.connect(self.clearAllAction);
        self.closeButton.clicked.connect(self.closeDialogAction);
        self.saveAsButton.clicked.connect(self.saveAsAction);
        
    #--------------------------- Public Methods ------------------

    @staticmethod
    def getInstance():
        '''
        Return the singleton instance of the PythonScriptDialog class.
        If no instance exists yet, it is created.
        @return: the singleton instance of the PythonScriptDialog class.
        @rtype: PythonScriptDialog
        '''
        if PythonScriptDialog.ui is not None:
            return PythonScriptDialog.ui;
        PythonScriptDialog.ui = PythonScriptDialog();
        return PythonScriptDialog.ui;

    def getPythonDisplayContent(self):
        '''
        Return the contents of the Python script result field as plain text.
        @return: All content of the python script result.
        @rtype: string
        '''
        return self.pythonDisplay.toPlainText();
    
    def setPythonDisplayContent(self, text):
        '''
        Fill the Python translation display with caller-provided text.
        @param text: plain text to place in the Python translation pane. Existing text is remoeved.
        @type text: string
        '''
        self.pythonDisplay.setPlainText(text);
        self.show();
        self.activateWindow();
        self.raise_();

    def convertOnePoseLineToPython(self, poseTableLine, poseNum=None, comment=None):
        '''
        Given a single Puppet pose, return a string with all the correspoinding
        Python script lines. 
        @param poseTableLine: line with all Puppet pose function, rarm(...);larm(...);head(...)...
        @type poseTableLine: string
        @param poseNum: Option sequence number of pose. If provided, a comment of the form
                        'Pose x:\n' is prepended to the series of Python statements that correspond
                        to the passed-in pose.
        @type poseNum: {int | string}
        @param comment: Option comment that will be prepended to the Python script statements.
        @type comment: string
        '''
        puppetCommands = poseTableLine.split(';');
        translation = "\n";
        if poseNum is not None:
            translation += "# Pose %s:\n" % str(poseNum);
        if comment is not None:
            translation += "# %s\n" % comment;
        
        for puppetCommand in puppetCommands:
            openParenPos = string.index(puppetCommand, '(');
            closeParenPos = string.index(puppetCommand, ')');
            args = puppetCommand[openParenPos + 1:closeParenPos];
            args = args.split(',');
            funcName = puppetCommand[:openParenPos];
            argStr = puppetCommand[openParenPos + 1:closeParenPos];
            try:
                if len(argStr) > 0:
                    translation += self.TRANSLATION_METHOD_MAP[funcName](*args);
                else:
                    translation += self.TRANSLATION_METHOD_MAP[funcName]();
            except KeyError:
                self.dialogService.showErrorMsg("Puppet function %s cannot be translated to Python." % funcName);
                continue;
        if string.find(poseTableLine, "rarm") > -1 and string.find(poseTableLine, "larm") > -1:
            translation += 'arm.wait_for(BOTH)\n';
        elif string.find(poseTableLine, "rarm") > -1:
            translation += 'arm.wait_for(RIGHT)\n';
        elif string.find(poseTableLine, "larm") > -1:
            translation += 'arm.wait_for(LEFT)\n';
        if string.find(poseTableLine, "lgrip") > -1 and string.find(poseTableLine, "rgrip") > -1:
            translation += 'gripper.wait_for(BOTH)\n';
        elif string.find(poseTableLine, "lgrip") > -1:
            translation += 'gripper.wait_for(LEFT)\n';
        elif string.find(poseTableLine, "rgrip") > -1:
            translation += 'gripper.wait_for(RIGHT)\n';
        # head.wait_for() exists in Python script, but it
        # hangs indefinitely; so we don't use it:
#        if string.find(poseTableLine, "head") > -1:
#            translation += 'head.wait_for()\n';
        # Torso does not have a wait_for:
#        if string.find(poseTableLine, "torso") > -1:
#            translation += 'torso.wait_for()\n';
        return translation;        
        
    #--------------------------- Private Methods ------------------
    
    def loadUIs(self):
        '''
        Loads the QtCreator XML file that defines the popup window for displaying 
        Python script translations.
        '''
        self.scriptDialog = QtBindingHelper.loadUi(os.path.join(QT_CREATOR_UI_FILE_ROOT, 
                                                                "pythonDialog/python_program_dialog.ui"),
                                                   self);
        self.setWindowTitle("Poses as Python");
        
    def copyAllAction(self):
        '''
        Bound to the 'Copy all' button of the translation window. Copies
        all the translation pane's contents to the X cut buffer. 
        '''
        self.pythonDisplay.selectAll();
        self.pythonDisplay.copy();
        
    def saveAsAction(self):
        '''
        Bound to the 'Save as' button of the translation window. Provides user
        with a file browser window for locating a desired destination directory
        and file name. Warns if file of same name already exists. Remembers the
        most recent Save-as directory, and initializes the subsequent file browser
        popups to begin with the latest saved-to directory. 
        '''
        # getSaveFileName() returns a tuple: (fileNames, selectedFilter).
        # We only allow selection of one file name, so only one filename is
        # returned. And we don't set filters, so we ignore the second part of
        # the returned couple:
        targetFile = QFileDialog.getSaveFileName(directory=self.recentDirSavedTo)[0];
        # Did they cancel out of the dialog?
        if len(targetFile) == 0:
            return;
        with open(targetFile, 'w') as fd:
            fd.write(self.pythonDisplay.toPlainText());
        # Next 'Save as' should open in the same directory: 
        self.recentDirSavedTo = os.path.dirname(targetFile);
        
    def clearAllAction(self):
        '''
        Bound to the 'Clear all' action. Removes all contents from the translation pane.
        '''
        self.pythonDisplay.clear();
        
    def closeDialogAction(self):
        '''
        Bound to the 'Close' button. Hides the translation popup window, but does 
        not destroy it.
        '''
        self.hide();
        
    def headToPython(self,pan, tilt):
        '''
        Generates a Python script head movement statement.
        @param pan: Head rotation in degrees 
        @type pan: {float | int}
        @param tilt: Head nod in degrees 
        @type tilt: {float | int}
        '''
        return "head.look_at(1.0,%s,%s)\n" % (pan, tilt);
    
    def rarmToPython(self, shoulderPan, shoulderLift, upperArmRoll, elbowFlex, forearmRoll, wristFlex, wristRoll):
        '''
        Generates Python script to move the right arm.
        @param shoulderPan: Joint value
        @type shoulderPan: float
        @param shoulderLift: Joint value
        @type shoulderLift: float
        @param upperArmRoll: Joint value
        @type upperArmRoll: float
        @param elbowFlex: Joint value
        @type elbowFlex: float
        @param forearmRoll: Joint value
        @type forearmRoll: float
        @param wristFlex: Joint value
        @type wristFlex: float
        @param wristRoll: Joint value
        @type wristRoll: float
        '''
        return "arm.move_to([%s,%s,%s,%s,%s,%s,%s],RIGHT)\n" % (shoulderPan, shoulderLift, upperArmRoll, elbowFlex, forearmRoll, wristFlex, wristRoll);

    def larmToPython(self, shoulderPan, shoulderLift, upperArmRoll, elbowFlex, forearmRoll, wristFlex, wristRoll):
        '''
        Generates Python script to move the left arm.
        @param shoulderPan: Joint value
        @type shoulderPan: float
        @param shoulderLift: Joint value
        @type shoulderLift: float
        @param upperArmRoll: Joint value
        @type upperArmRoll: float
        @param elbowFlex: Joint value
        @type elbowFlex: float
        @param forearmRoll: Joint value
        @type forearmRoll: float
        @param wristFlex: Joint value
        @type wristFlex: float
        @param wristRoll: Joint value
        @type wristRoll: float
        '''
        return "arm.move_to([%s,%s,%s,%s,%s,%s,%s],LEFT)\n" % (shoulderPan, shoulderLift, upperArmRoll, elbowFlex, forearmRoll, wristFlex, wristRoll);
    
    def rgripToPython(self, gripperWidth):
        '''
        Generates Python script for setting the right gripper jaw width.
        @param gripperWidth: Target gripper width in cm.
        @type gripperWidth: {int | float}
        '''
        if float(gripperWidth) > 0.0:
            return "gripper.rel(RIGHT)\n";
        else:
            return "gripper.close(RIGHT)\n"; 
        
    def lgripToPython(self, gripperWidth):
        '''
        Generates Python script for setting the left gripper jaw width.
        @param gripperWidth: Target gripper width in cm.
        @type gripperWidth: {int | float}
        '''
        if float(gripperWidth) > 0:
            return "gripper.rel(LEFT)\n";
        else:
            return "gripper.close(LEFT)\n";
        
    def torsoToPython(self, height):
        '''
        Generates Python script for lifting and lowering the torso.
        @param height: Target height in meters.
        @type height: {string | int | float}
        '''
        # Convert from the Puppet convention of cm to the Python script convention
        # of meters:
        try:
            height = float(height);
        except ValueError:
            raise ValueError("Torso height must be convertible to a float. But %s is not." % str(height));
             
        return "torso.set(%s)\n" % str(height / 100.0);
    
    def speakToPython(self, text, voice=None):
        '''
        Generates Python script for uttering text.
        @param text: The words for the robot to say.
        @type text: string
        @param voice: The text-to-speech voice to use. Default is the 
                      built-in Ubuntu computerish sounding voice. Current other
                      options are David, Amy, Shouty, and Whispery. Their
                      availability depends on licensing on the code's target
                      machine.
        @type voice:
        '''
        try:
            # Is parameter 'text' of form text='Foo'. String the 'text=':
            text = text[string.index(text,'=') + 1:];
        except ValueError:
            # text is just the words:
            pass
        return "sound.say(%s)\n" % text;

    def lookAtFaceToPython(self):
        '''
        Generate Python script for the look-at-face feature.
        '''
        #self.dialogService.showInfoMessage("Python scripts do not have 'Look-at-Face. Head will turn to look forward.");
        #return "head.look_at(1.0, 0.0, 1.0)\n";
        return "head.look_at_face()\n";
    

    def shutdown(self):
        '''
        Cleanly destroy the translation window, whether or not it is currently
        visible to the user.
        '''
        self.close();

#------------------------------ Class DialogService ----------------------------
    
class DialogService(object):
    '''
    Convenience class for popping up error and information messages
    in Qt based applications. Control whether any popup windows are
    actually shown by setting the class variable CURRENTLY_TESTING to
    True or False. If True, no dialogs will be shown. Used during 
    automatic testing.
    '''

    CURRENTLY_TESTING = False;
    
    #----------------------------------
    # Initializer
    #--------------

    def __init__(self, parent=None):
        
        # All-purpose error popup message:
        # Used by self.showErrorMsgByErrorCode(<errorCode>), 
        # or self.showErrorMsg(<string>). Returns a
        # QErrorMessage without parent, but with QWindowFlags set
	    # properly to be a dialog popup box:
        self.errorMsgPopup = QErrorMessage.qtHandler();
       	# Re-parent the popup, retaining the window flags set
        # by the qtHandler:
        self.errorMsgPopup.setParent(parent, self.errorMsgPopup.windowFlags());
        #self.errorMsgPopup.setStyleSheet(SpeakEasyGUI.stylesheetAppBG);
        
        self.infoMsg = QMessageBox(parent=parent);
        self.infoMsg.setStyleSheet("background-color : rgb(160,182,191)");
        #self.infoMsg.setStyleSheet(SpeakEasyGUI.stylesheetAppBG);
    
    #----------------------------------
    # showErrorMsg
    #--------------
    QErrorMessage
    def showErrorMsg(self,errMsg):
        '''
        Given a string, pop up an error dialog, if DialogService.CURRENTLY_TESTING is False.
        Else method returns quietly, because unittesting is under way
        @param errMsg: The message
        @type errMsg: string
        '''
        if not DialogService.CURRENTLY_TESTING:
            self.errorMsgPopup.showMessage(errMsg);
    
    #----------------------------------
    # showInfoMsg 
    #--------------

    def showInfoMessage(self, text):
        '''
        Given a string, pop up an info message, if DialogService.CURRENTLY_TESTING is False.
        Else method returns quietly, because unittesting is under way
        @param text: The message
        @type text: string
        '''
        if not DialogService.CURRENTLY_TESTING:
            self.infoMsg.setText(text);
            self.infoMsg.exec_();        
    
    # ---------------------------- Testing --------------------
        
if __name__ == "__main__":
    
    # Testing
    
    import unittest;
    
    app = QApplication(sys.argv);
    pythonScriptDialog = PythonScriptDialog();

    #----------------------
    # UI functionality:
    #---------
    
#    pythonScriptDialog.show();
#    # Enter Qt application main loop
#    sys.exit(app.exec_());

    #----------------------
    # Puppet to Python Conversion  Functionality:
    #---------
    class TestToPythonConversion(unittest.TestCase):
        '''
        Unittest for code in this file. Just run.
        '''
        
        def setUp(self):
            # Ensure that no error dialog boxes are raised. That would
            # interrupt the test automation:
            DialogService.CURRENTLY_TESTING = True;
            
        def tearDown(self):
            DialogService.CURRENTLY_TESTING = False;
        
        def testTorso(self):
            xlation = pythonScriptDialog.convertOnePoseLineToPython("torso(3.0)");
            self.assertEqual(xlation, "\ntorso.set(0.03)\n", "Torso translation failed. Was '%s'." % xlation);
            xlation = pythonScriptDialog.convertOnePoseLineToPython("torso(3.0)", poseNum=1);
            self.assertEqual(xlation, "\n# Pose 1:\ntorso.set(0.03)\n", "Torso translation with pose num failed. Was '%s'" % xlation);
            xlation = pythonScriptDialog.convertOnePoseLineToPython("torso(3.0)", poseNum=1, comment="Fly like a bird");
            self.assertEqual(xlation, "\n# Pose 1:\n# Fly like a bird\ntorso.set(0.03)\n", "Torso translation with pose num failed. Was '%s'" % xlation);
            
        def testHead(self):
            xlation = pythonScriptDialog.convertOnePoseLineToPython("head(-38,30)");
            self.assertEqual(xlation, "\nhead.look_at(1.0,-38,30)\n", "Head translation failed. Was '%s'." % xlation);
    
        def testRArm(self):
            xlation = pythonScriptDialog.convertOnePoseLineToPython("rarm(1,2,3,4,5,6,7)");
            self.assertEqual(xlation, "\narm.move_to([1,2,3,4,5,6,7],RIGHT)\narm.wait_for(RIGHT)\n", "Right arm translation failed. Was '%s'." % xlation);
            
        def testLArm(self):
            xlation = pythonScriptDialog.convertOnePoseLineToPython("larm(-1,-2,-3,-4,-5,-6,-7)");
            self.assertEqual(xlation, "\narm.move_to([-1,-2,-3,-4,-5,-6,-7],LEFT)\narm.wait_for(LEFT)\n", "Left arm translation failed. Was '%s'." % xlation);

        def testBothArms(self):
            xlation = pythonScriptDialog.convertOnePoseLineToPython("larm(-1,-2,-3,-4,-5,-6,-7);rarm(1,2,3,4,5,6,7)");
            self.assertEqual(xlation, "\narm.move_to([-1,-2,-3,-4,-5,-6,-7],LEFT)\narm.move_to([1,2,3,4,5,6,7],RIGHT)\narm.wait_for(BOTH)\n", 
                             "Both arms translation failed. Was '%s'." % xlation);
        def testRGripper(self):
            xlation = pythonScriptDialog.convertOnePoseLineToPython("rgrip(3.0)");
            self.assertEqual(xlation, "\ngripper.rel(RIGHT)\ngripper.wait_for(RIGHT)\n", "Right gripper open translation failed. Was '%s'." % xlation);
            xlation = pythonScriptDialog.convertOnePoseLineToPython("rgrip(0)");
            self.assertEqual(xlation, "\ngripper.close(RIGHT)\ngripper.wait_for(RIGHT)\n", "Right gripper close translation failed. Was '%s'." % xlation);

        def testLGripper(self):
            xlation = pythonScriptDialog.convertOnePoseLineToPython("lgrip(3.0)");
            self.assertEqual(xlation, "\ngripper.rel(LEFT)\ngripper.wait_for(LEFT)\n", "Left gripper open translation failed. Was '%s'." % xlation);
            xlation = pythonScriptDialog.convertOnePoseLineToPython("lgrip(0)");
            self.assertEqual(xlation, "\ngripper.close(LEFT)\ngripper.wait_for(LEFT)\n", "Left gripper close translation failed. Was '%s'." % xlation);

        def testBothGrippers(self):
            xlation = pythonScriptDialog.convertOnePoseLineToPython("lgrip(3.0);rgrip(5.0)");
            self.assertEqual(xlation, "\ngripper.rel(LEFT)\ngripper.rel(RIGHT)\ngripper.wait_for(BOTH)\n", "Both gripper open translation failed. Was '%s'." % xlation);
            
            xlation = pythonScriptDialog.convertOnePoseLineToPython("lgrip(0);rgrip(0)");
            self.assertEqual(xlation, "\ngripper.close(LEFT)\ngripper.close(RIGHT)\ngripper.wait_for(BOTH)\n", "Both gripper close translation failed. Was '%s'." % xlation);

            xlation = pythonScriptDialog.convertOnePoseLineToPython("lgrip(6.0);rgrip(0)");
            self.assertEqual(xlation, "\ngripper.rel(LEFT)\ngripper.close(RIGHT)\ngripper.wait_for(BOTH)\n", 
                             "Left gripper open, right gripper close translation failed. Was '%s'." % xlation);

        def testSpeak(self):
            xlation = pythonScriptDialog.convertOnePoseLineToPython("speak(text='Foobar', voice='David')");
            self.assertEqual(xlation, "\nsound.say('Foobar')\n", "Speak translation failed. Was '%s'." % xlation);
            xlation = pythonScriptDialog.convertOnePoseLineToPython("speak('Foobar')");
            self.assertEqual(xlation, "\nsound.say('Foobar')\n", "Speak translation short form failed. Was '%s'." % xlation);
            
        def testLookAtFace(self):
            xlation = pythonScriptDialog.convertOnePoseLineToPython("look_at_face()");
            self.assertEqual(xlation, "\nhead.look_at(1.0, 0.0, 1.0)\n", "Look-at-face translation failed. Was '%s'." % xlation);
            

    unittest.main();
