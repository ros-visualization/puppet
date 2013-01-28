#!/usr/bin/env python

import roslib; roslib.load_manifest('slider_gui');
import rospy;

from functools import partial;
import sys;
import os;
import copy;
import threading;

import rospy;

from python_qt_binding import QtBindingHelper;
from PyQt4.QtGui import QMainWindow, QApplication, QSlider, QDial, QPushButton;
from PyQt4 import QtCore;

from sensor_msgs.msg import Joy

from slider_gui.pythonScriptDialog import DialogService;

QT_CREATOR_UI_FILE_ROOT = os.path.join(os.path.dirname(os.path.realpath(__file__)), "../src/QtCreatorFiles");
# The index of the mode button in Joy message button array:
SLIDER_MODE_BUTTON_JOY_MSG_INDEX = 24;

#TX_BLACKOUT_DURATION = rospy.Duration.from_sec(0.1);
TX_BLACKOUT_DURATION = 0.1;

class AccessibleSlider(QMainWindow):
    
    # Number of slider/knob combinations. Change this number 
    # if you add slider/knob sets to the GUI. The program will
    # take them into account:
    NUM_SLIDER_BOARD_CHANNELS = 9;
    # Slider and dial min/max: While clients of this class work
    # with floats between -1.0 and 1.0, the sliders themselves
    # can only be integer values. So the following min/max nums
    # are scaled by 100. Pecision beyond the second place
    # is rounded and lost beyond that:
    SLIDER_MAX_VAL = 1.0;
    SLIDER_MIN_VAL = -1.0;
    DIAL_MAX_VAL = SLIDER_MAX_VAL;
    DIAL_MIN_VAL = SLIDER_MIN_VAL;
    
    # Controlling flow of incoming Joy messages for mirroring
    # the sliders:
    joyMsgLock = threading.Lock();
    
    # Controlling flow of Qt soft slider/dial signals:
    sliderSigsLock = threading.Lock();
    
    # Singleton instance, obtainable via this
    # class var.
    ui = None;
    #--------------------------- Public Methods ------------------
    
        
    def __init__(self, parent=None):
        
        # Enforce singleton op for this class. 
        # We could simply return, if we detect
        # a second instantiation here, but it seems
        # better to clarify the correct operation: 
        if AccessibleSlider.ui is not None:
            raise RuntimeError("Can only have single instance of AccessibleSlider. Use getInstance() factory method.");
        
        # For accessibility: keep sliders on top:
        super(AccessibleSlider, self).__init__(parent, QtCore.Qt.WindowStaysOnTopHint);
        self.dialogService = DialogService();

        # Set of slider/dial that were changed between
        # transmission blackouts:
        self.sliderDialSet = set();

        self.initUI();
        # Move this window to the center of the screen,
        # else it always pops up far away:
        self.move(QApplication.desktop().screen().rect().center() - self.rect().center())
        
        self.joyMsg = Joy();
        # One axis for each slider and each dial (the factor of 2):
        self.joyMsg.axes = [ 0 ] * AccessibleSlider.NUM_SLIDER_BOARD_CHANNELS * 2;
        # Buttons: 18 channel buttons + 4 tape recorder buttons + 1 scene button:
        self.joyMsg.buttons = [ 0 ] * 25
        mode = None

        rospy.init_node('accessibleSliders')
        self.pub = rospy.Publisher('joy', Joy, latch=True)
        rospy.Subscriber('/joy', Joy, self.joyCallback)
        
        # Timer for checking on stacked up slider events:
        #self._timer = None;
        #self._timer = rospy.Timer(TX_BLACKOUT_DURATION, self.serviceDialSliderActions, oneshot=True);
        #self._timer = rospy.Timer(TX_BLACKOUT_DURATION, self.serviceDialSliderActions);
        self._timer = threading.Timer(TX_BLACKOUT_DURATION, self.serviceDialSliderActions);
        self._timer.start();
        
        self.show();

    def shutdown(self):
        self._timer.shutdown();

    def setSliderValue(self, channelIndex, value):
        self.checkChannelIndexInRange(channelIndex);
        scaledValue = value * 100.0; 
        self.checkSliderAndDialValueInRange(value);
        # Sliders can only be integers:
        self.sliders[channelIndex].setSliderPosition(int(scaledValue));
        
    def setSliderValues(self, valueArr):
        if len(valueArr) > len(self.sliders):
            raise ValueError("Only %d sliders are available, but %d values were passed in." % (AccessibleSlider.NUM_SLIDER_BOARD_CHANNELS, len(valueArr)));
        for i,val in enumerate(valueArr):
            self.setSliderValue(i, val)
        
    def getSliderValue(self, channelIndex):
        self.checkChannelIndexInRange(channelIndex);
        # Sliders can only be integers, but we want range to be -1 to +1:
        self.sliders[channelIndex].value(float(value / 100.0));
    
    def getDialValue(self, channelIndex):
        self.checkChannelIndexInRange(channelIndex);
        # Dials can only be integers, but we want range to be -1 to +1:
        self.dials[channelIndex].value(float(value / 100.0));
        
    def setDialValue(self, channelIndex, value):
        self.checkChannelIndexInRange(channelIndex);
        scaledValue = value * 100.0; 
        self.checkSliderAndDialValueInRange(value);
        # Dials can only be integers:
        self.dials[channelIndex].setSliderPosition(int(scaledValue));
        
    def setDialValues(self, valueArr):
        if len(valueArr) > len(self.dials):
            raise ValueError("Only %d dials are available, but %d values were passed in." % (AccessibleSlider.NUM_SLIDER_BOARD_CHANNELS, len(valueArr)));
        for i,val in enumerate(valueArr):
            self.setDialValue(i, val);
    
    def readSliderAndDialValues(self):
        vals = [];
        for i in range(len(self.sliders)): 
            vals.append(self.getSliderValue(i));
        for i in range(len(self.sliders), len(self.sliders) + len(self.dials)):
            vals.append(self.getdialValue(i));
        return vals;        
    
    def readButtonValues(self):
        vals = [];
        for i in range(AccessibleSlider.NUM_SLIDER_BOARD_CHANNELS * 2):
            vals.append(self.channelButtons[i].value());
        vals.append(self.rewindButton.value())
        vals.append(self.playButton.value())
        vals.append(self.forwardButton.value())
        vals.append(self.againButton.value())
        vals.append(self.stopButton.value())
        vals.append(self.recordButton.value())
        
        # The motion modes are radio buttons. We translate
        # them into a number between 0 and 3 for the joy message:
        vals.append(self.getMotionMode());
        
    def getMotionMode(self):
        if self.symmetricRadioButton.value == 1:
            return 0;
        elif self.leftRadioButton.value == 1:
            return 1;
        elif self.rightRadioButton == 1:
            return 2;
        elif self.baseRadioButton == 1:
            return 3;
    
    def setDialLabel(self, channelIndex, label):
        self.checkChannelIndexInRange(channelIndex);
        self.dialLabels[channelIndex].text = label;
        
    def setDialLabels(self, labelArr):
        if len(labelArr > len(self.dialLabels)):
            raise ValueError("Slider board has %d channels. Must pass in no more than that many labels. Instead passed in %d." % len(labelArr));
        for i in range(len(labelArr)):
            setDialLabel(i, labelArr[i]);
        
    def checkChannelIndexInRange(self, channelIndex):
        if (channelIndex < 0) or (channelIndex > AccessibleSlider.NUM_SLIDER_BOARD_CHANNELS):
            raise ValueError("Slider board has %d channels. Channel index must be between 0 and that %d") % (AccessibleSlider.NUM_SLIDER_BOARD_CHANNELS,
                                                                                                             AccessibleSlider.NUM_SLIDER_BOARD_CHANNELS-1);
    def checkSliderAndDialValueInRange(self, value):
        if (value < AccessibleSlider.SLIDER_MIN_VAL) or (value >AccessibleSlider.SLIDER_MAX_VAL):
            raise ValueError("Slider and dial values must be between %0.f and %0.f. Was %0.f" % (AccessibleSlider.SLIDER_MAX_VAL,
                                                                                                 AccessibleSlider.SLIDER_MIN_VAL,
                                                                                                 value));
            
    #--------------------------- Private Methods ------------------

    def initUI(self):
        '''
        Create convenient button names, and connect buttons to actions.
        '''
        
        self.setWindowTitle("Slider Board");
        
        # Load QtCreator's XML UI files:
        # Make QtCreator generated UIs children if this instance:
        self.loadUIs();
        self.dials           = [];
        self.sliders         = [];
        self.channelButtons  = [];
        self.joyAxisValues   = [];
        self.joyButtonValues = [];
        self.dialLabels      = [];
        self.buttonPosLookup = {};
        self.dialSliderPosLookup = {};
        # All knobs:
        for i in range(AccessibleSlider.NUM_SLIDER_BOARD_CHANNELS):
            dialName = 'd%dDial' % i;
            dialLabelName = 'knob%dLabel' % i;
            sliderName = 's%dSlider' % i;
            channelButtonNameA = 'ch%daButton' % i;
            channelButtonNameB = 'ch%dbButton' % i;
            
            self.dials.append(getattr(self.sliderBoard, dialName));
            
            self.dialLabels.append(getattr(self.sliderBoard, dialLabelName));
            
            self.sliders.append(getattr(self.sliderBoard, sliderName));
            
            self.channelButtons.append(getattr(self.sliderBoard, channelButtonNameA));
            self.channelButtons.append(getattr(self.sliderBoard, channelButtonNameB));
            # Lookup for button obj ==> axis place in joy msg:
            self.buttonPosLookup[self.channelButtons[-2]] = 2*i;
            self.buttonPosLookup[self.channelButtons[-1]] = 2*i+1;
            
            #self.dials[-1].valueChanged.connect(partial(self.dialsAction, self.dials[-1]), i);
            self.dials[-1].valueChanged.connect(partial(self.dialOrSliderAction, self.dials[-1]));
            self.sliders[-1].valueChanged.connect(partial(self.dialOrSliderAction, self.sliders[-1]));
            
            self.channelButtons[-2].pressed.connect(partial(self.channelButtonsAction, self.channelButtons[-2]));
            self.channelButtons[-2].released.connect(partial(self.channelButtonsAction, self.channelButtons[-2]));
            self.channelButtons[-1].pressed.connect(partial(self.channelButtonsAction, self.channelButtons[-1]));
            self.channelButtons[-1].released.connect(partial(self.channelButtonsAction, self.channelButtons[-1]));
                                           

        # Create lookup mapping slider or dial objects to indexes in 
        # the Joy message axes array:
        for i,slider in enumerate(self.sliders):
            self.dialSliderPosLookup[slider] = i;
        for i,dial in enumerate(self.dials):
            self.dialSliderPosLookup[dial] = len(self.sliders) + i;
                    
        self.rewindButton  = self.sliderBoard.rewindButton;
        self.buttonPosLookup[self.rewindButton] = 2*AccessibleSlider.NUM_SLIDER_BOARD_CHANNELS;
        self.playButton    = self.sliderBoard.playButton;
        self.buttonPosLookup[self.playButton] = 2*AccessibleSlider.NUM_SLIDER_BOARD_CHANNELS + 1;
        self.forwardButton = self.sliderBoard.forwardButton;
        self.buttonPosLookup[self.forwardButton] = 2*AccessibleSlider.NUM_SLIDER_BOARD_CHANNELS + 2;
        self.againButton  = self.sliderBoard.againButton;
        self.buttonPosLookup[self.againButton] = 2*AccessibleSlider.NUM_SLIDER_BOARD_CHANNELS + 3;
        self.stopButton    = self.sliderBoard.stopButton;
        self.buttonPosLookup[self.stopButton] = 2*AccessibleSlider.NUM_SLIDER_BOARD_CHANNELS + 4;
        self.recordButton  = self.sliderBoard.recordButton;
        self.buttonPosLookup[self.recordButton] = 2*AccessibleSlider.NUM_SLIDER_BOARD_CHANNELS + 5;
        
        self.symmetricRadioButton = self.sliderBoard.symmetricRadioButton;
        self.leftRadioButton      = self.sliderBoard.leftRadioButton;
        self.rightRadioButton     = self.sliderBoard.rightRadioButton;
        self.baseRadioButton      = self.sliderBoard.baseRadioButton;
        
        self.rewindButton.pressed.connect(partial(self.recorderButtonsAction, self.rewindButton));
        self.rewindButton.released.connect(partial(self.recorderButtonsAction, self.rewindButton));
        self.playButton.pressed.connect(partial(self.recorderButtonsAction, self.playButton));
        self.playButton.released.connect(partial(self.recorderButtonsAction, self.playButton));
        self.forwardButton.pressed.connect(partial(self.recorderButtonsAction, self.forwardButton));
        self.forwardButton.released.connect(partial(self.recorderButtonsAction, self.forwardButton));
        self.againButton.pressed.connect(partial(self.recorderButtonsAction, self.againButton));
        self.againButton.released.connect(partial(self.recorderButtonsAction, self.againButton));
        self.stopButton.pressed.connect(partial(self.recorderButtonsAction, self.stopButton));
        self.stopButton.released.connect(partial(self.recorderButtonsAction, self.stopButton));
        self.recordButton.pressed.connect(partial(self.recorderButtonsAction, self.recordButton));
        self.recordButton.released.connect(partial(self.recorderButtonsAction, self.recordButton));
        
        self.symmetricRadioButton.clicked.connect(partial(self.modeButtonsAction, self.symmetricRadioButton));
        self.leftRadioButton.clicked.connect(partial(self.modeButtonsAction, self.leftRadioButton));
        self.rightRadioButton.clicked.connect(partial(self.modeButtonsAction, self.rightRadioButton));
        self.baseRadioButton.clicked.connect(partial(self.modeButtonsAction, self.baseRadioButton));

        # Place to retain slider values:
        #self.setSliderValues([0.0]*AccessibleSlider.NUM_SLIDER_BOARD_CHANNELS);
        #self.setDialValues([0.0]*AccessibleSlider.NUM_SLIDER_BOARD_CHANNELS);

        # For convenience, get one big array with slider objs, followed
        # by dial objs:
        self.sliderDials = copy.copy(self.sliders);
        self.sliderDials.extend(self.dials);
        
        # For convenience: get one big array of all button objects,
        # including the recorder buttons:
        self.allButtonObjs = copy.copy(self.channelButtons);
        self.allButtonObjs.extend([self.rewindButton, self.playButton, self.forwardButton, 
                                   self.againButton, self.stopButton,self.recordButton]);
        
    def loadUIs(self):
        '''
        Loads the QtCreator XML file that defines the slider board.
        '''
        self.sliderBoard = QtBindingHelper.loadUi(os.path.join(QT_CREATOR_UI_FILE_ROOT, 
                                                                "sliderBoard/slider_board.ui"),
                                                   self);

    def dialOrSliderAction(self, dialOrSlider):
        # Remember that this dial or slider changed:
        self.queueSliderEvent(dialOrSlider);
        # Adjust the slider position without taking any action:
        dialOrSlider.setValue(dialOrSlider.value());
        return;
        
    def serviceDialSliderActions(self):
        
        #self._timer = None
        
        # Savely get an array of all slider or dial objects that changed
        # since the beginning of the transmission blackout:
        allSlidersDials = self.dequeueSliderEvents();
        
        try:
            for dialOrSlider in allSlidersDials:
                # Use latest value as target joint angle:
                newVal = dialOrSlider.value();
                self.joyMsg.axes[self.dialSliderPosLookup[dialOrSlider]] = newVal / 100.0;
                self.joyMsg.header.stamp = rospy.Time.now();
                self.publishTheJoy();
        finally:
            #self._timer = rospy.Timer(TX_BLACKOUT_DURATION, self.serviceDialSliderActions, oneshot=True);
            self._timer = threading.Timer(TX_BLACKOUT_DURATION, self.serviceDialSliderActions);
            self._timer.start();
    
    def channelButtonsAction(self, channelButton):
        if channelButton.isDown():
            self.joyMsg.buttons[self.buttonPosLookup[channelButton]] = 1;
        else:
            self.joyMsg.buttons[self.buttonPosLookup[channelButton]] = 0;
        self.publishTheJoy();
    
    def recorderButtonsAction(self, recorderButton):
        if recorderButton.isDown():
            self.joyMsg.buttons[self.buttonPosLookup[recorderButton]] = 1;
        else:
            self.joyMsg.buttons[self.buttonPosLookup[recorderButton]] = 0;
        self.publishTheJoy();

    def modeButtonsAction(self, modeRadioButton):
        if modeRadioButton == self.symmetricRadioButton:
            self.joyMsg.buttons[SLIDER_MODE_BUTTON_JOY_MSG_INDEX] = 0;
        elif modeRadioButton == self.leftRadioButton:
            self.joyMsg.buttons[SLIDER_MODE_BUTTON_JOY_MSG_INDEX] = 1;
        elif modeRadioButton == self.rightRadioButton:
            self.joyMsg.buttons[SLIDER_MODE_BUTTON_JOY_MSG_INDEX] = 2;
        elif modeRadioButton == self.baseRadioButton:
            self.joyMsg.buttons[SLIDER_MODE_BUTTON_JOY_MSG_INDEX] = 3;
        self.publishTheJoy();

    def publishTheJoy(self):
        self.pub.publish(self.joyMsg);
        
        
    def joyCallback(self, joyMsg):
        with AccessibleSlider.joyMsgLock:
            for i,axisValue in enumerate(joyMsg.axes):
                self.sliderDials[i].setValue(axisValue * 100);
                self.joyMsg.axes[i] = axisValue;
            # Adjust all channel buttons to match the 1/0 setting in the
            # received Joy message:
            #for i,buttonValue in enumerate(joyMsg.buttons[0:5+2*AccessibleSlider.NUM_SLIDER_BOARD_CHANNELS]):
            for i,buttonValue in enumerate(joyMsg.buttons[0:len(self.allButtonObjs)]):
                self.allButtonObjs[i].setDown(buttonValue == 1);
                self.joyMsg.buttons[i] = buttonValue == 1;
            
            # Get the Mode button state (0,1,2,3):
            modeButton = joyMsg.buttons[SLIDER_MODE_BUTTON_JOY_MSG_INDEX];
            if modeButton == 0:
                self.symmetricRadioButton.setDown(1);
            elif modeButton == 1:
                self.leftRadioButton.setDown(1);
            elif modeButton == 1:
                self.rightRadioButton.setDown(1);
            else:
                self.baseRadioButton.setDown(1);
    
    def queueSliderEvent(self, sliderOrDialObj):
        '''
        Safely collect slider and/or dial objects into a set.
        Those instances have changed value while system was
        in transmission blackout period.
        @param sliderOrDialObj: instance to be saved.
        @type sliderOrDialObj: {QSlider | QDial}
        '''
        with AccessibleSlider.sliderSigsLock:
            self.sliderDialSet.add(sliderOrDialObj);
            
    def dequeueSliderEvents(self):
        '''
        Safely obtain an array of slider or dial objects that
        have changed during the transmission blackout:
        @return: array of Qt slider and dial instances:
        @rtype: [{QSlider | QDial}] 
        '''
        with AccessibleSlider.sliderSigsLock:
            sliderDialObjs = [];
            try:
                while 1:
                    sliderDialObjs.append(self.sliderDialSet.pop());
            except KeyError:
                return sliderDialObjs;
                
                    
    
if __name__ == '__main__':
    
    app = QApplication(sys.argv);
    slider = AccessibleSlider();
    try:
        sys.exit(app.exec_())
    except KeyboardInterrupt, ROSInterruptException:
        slider.shutdown();
        sys.exit();                
