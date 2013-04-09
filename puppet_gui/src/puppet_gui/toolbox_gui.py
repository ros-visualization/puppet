#!/usr/bin/env python

'''
This module is a horrible mess, as is its companion toolbox.py. Both consist
of pieces of tapper.py ripped out in a hurry, and placed into these two files.
The extensive commented-out code is to make it easier to add the music component
later. It is present in tapper.py.
'''



import os
import sys
import threading
import time
import random
from functools import partial

import roslib
roslib.load_manifest('speakeasy')
import rospy

from python_qt_binding import loadUi
from python_qt_binding import QtGui, QtCore
from QtGui import QApplication, QDialog, QMainWindow
from QtCore import Signal,QObject 

from speakeasy.speakeasy_ui import DialogService;


class CommChannel(QObject):
    '''
    Combines signals into one place.
    '''
    pauseBlinkSignal         = Signal();
    playCounterUpdateSignal  = Signal();
    
    # Signals to Puppet UI for inserting values into the UI:
    
    # Music related:
    insertTimeDurationSignal = Signal(float);
    # Signal to insert song start: songName, startAt, repeats, playFor:
    insertStartSongSignal    = Signal('QString', float, int, float, bool);
    insertPauseSongSignal   = Signal();
    insertUnPauseSongSignal = Signal();
    insertStopSongSignal    = Signal();
    
    # Sound effect related:
    
    # number of repeats after initial play; -1 if forever
    insertStartSoundSignal   = Signal('QString', int, bool);
    insertPauseSoundSignal   = Signal();
    insertUnPauseSoundSignal = Signal();
    insertStopSoundSignal    = Signal();
    
    # Wait signal:
    insertWaitSignal         = Signal(float);
    
    # Text-to-speech related:
    insertRobotSaysSignal    = Signal('QString', 'QString', 'QString'); # What to say; voice to use; ttsEngine

class PlayPauseIconState:
    '''
    Enum expressing whether the Play/Pause combo button is
    currently showing a Paused or a Playing icon.
    '''
    PAUSE_ICON_SHOWING = 0;
    PLAY_ICON_SHOWING  = 1;

class SoundType:
    '''
    Enum marking a variable as music, or a sound effect.
    '''
    SONG = 0;
    SOUND_EFFECT = 1;

class ToolboxGUI(QDialog):
    '''
    Class for adding music, sound, and text-to-speech commands to the Puppet application.
    The class creates a window with several tabs. Each tabs contains a coherent set
    of related activities. The window is popped up by the main Puppet application,
    and hidden (minimized) by the user if desired. The main effect of user interactions
    with Tapper is to insert pose change timings and music/sound/speech start, pause,
    unpause, and stop commands into the Puppet script execution flow. In particular
    facilities include:
    
        1. Exploring the music collection, playing snippets of the available songs.
        2. Tapping or just entering the time base of a chosen song in Beats per Minute (BPM),
        3. Computing pose durations in terms of song time base.
        4. Adding play, pause, unpause, and stop commands to pose execution sequences.
        5. Entering text that is to be uttered by the robot, or locally by the computer
           that is running the Puppet application.
    This class is a singleton. Use the getInstance() method to obtain an instance.
    '''
    
    # Singleton instance, obtainable via this
    # class var.
    inst = None;
    
#    WINDOW_SIZE = QRect(88, 682, 800, 500)
    # Radio button minimum font size:
    RADIO_BUTTON_LABEL_FONT_SIZE = 16; # pixels
    TAP_LABEL_FONT_SIZE = 48; # pixels
    
    TAB_INDEX_PICK_SONG = 0;
    TAB_INDEX_TAP_BEAT  = 1;
    TAB_INDEX_USE_BEAT  = 2;
    
    EMPTY_SONG_LIST_TEXT = "No songs found";
    EMPTY_SOUND_LIST_TEXT = "No sound effects found";
    
    def __init__(self, rootWindow, toolbox):
        '''
        Start a new Tapper instance.
        @param stand_alone: True if no ROS node is to be used, but operation is
                            to use local sound/speech facilities. If a ROS node
                            is intended to be used to generate the sounds, make
                            this False. Set to None if ROS node is to be used
                            when available, using local operation as fallback.
        @param closable: Set True if window is to be closable via the window frame
                            cross. Setting to False prevents window close until
                            <tapperInstance>.closable = True is issued.
        '''
        # Enforce singleton op for this class. 
        # We could simply return, if we detect
        # a second instantiation here, but it seems
        # better to clarify the correct operation: 
        if ToolboxGUI.inst is not None:
            raise RuntimeError("Can only have single instance of ToolboxGUI. Use getInstance() factory method.");
        
        super(ToolboxGUI, self).__init__();
        
        self.rootWindow = rootWindow;
        self.toolbox = toolbox;
        self.textToSpeechProvider = toolbox.textToSpeechProvider;
        
        # Need directory where sound effects and music are kept.
        # First, try the parameter server. The launch file sets
        # parameter "speakeasy/sounds_dir":
#        try:
#            self.SOUND_DIRECTORY = rospy.get_param("speakeasy/sounds_dir");
#        except KeyError:
#            # If parameter not set, try to find the speakeasy package,
#            # and assume subdirectory 'sounds"
#            speakEasyPackageDir = SpeakeasyUtils.findPackage('speakeasy');
#            if speakEasyPackageDir is not None:
#                self.SOUND_DIRECTORY = os.path.join(speakEasyPackageDir, "sounds");
#                if not os.path.exists(self.SOUND_DIRECTORY):
#                    raise NotImplementedError("The SpeakEasy sounds directory was not found in parameter speakeasy/sounds. " +
#                                              "Nor was a subdirectory 'sounds' found in %s." % self.SOUND_DIRECTORY); 
#                    
#            else:
#                raise NotImplementedError("The SpeakEasy sounds directory was not found in parameter speakeasy/sounds, nor was package speakeasy found.");
#        self.SONG_DIRECTORY = os.path.join(self.SOUND_DIRECTORY, "music");
        
        ToolboxGUI.inst = self;
        self.commChannel = CommChannel();
        # During unittest there is no GUI. We recognize that by the rootWindow being None:
        self.dialogService = DialogService();
        self.initUI();
        
        # Initialize tapping panel data:
#        self.resetTap();
        
        # Dictionary to remember full beat (and in future other properties)
        # for each song:
#        self.songBeats = {};
        
        # Start thread that blinks the pause button when
        # paused, and updates the playhead counter while
        # playing: 
#        self.maintenanceThread = OneSecondMaintenance(self, 
#                                                      self.musicPlayer, 
#                                                      self.commChannel.pauseBlinkSignal, 
#                                                      self.commChannel.playCounterUpdateSignal);
#        self.maintenanceThread.start();


    @staticmethod
    def getInstance(rootWindow, toolbox):
        '''
        Return a singleton Tapper instance, creating it if needed.
        @param stand_alone: Whether the music features are being used locally, or
                            whether a SpeakEasy node is running elsewhere. Default
                            is to look for a ROS node, and switch to local op if no
                            node is found. 
        @type stand_alone: boolean
        @param closable: Controls whether users are allowed to close the Tapper window
                         by clicking on the cross icon on the window title bar. 
        @type closable: boolean
        '''
        if ToolboxGUI.inst is not None:
            return ToolboxGUI.inst;
        if rootWindow is None or toolbox is None:
            raise ValueError("Both rootWindow and toolbox must not be provided. One or the other was None")
        return ToolboxGUI(rootWindow, toolbox);

    def shutdown(self):
        '''
        Cleanly destroy the Tapper window.
        '''
        self.close();
        
    def initUI(self):
        '''
        Created convenient instance variables for QtCreator generated UI elements.
        '''
        # Make convenient names for the nested widgets:
        self.voiceComboBox = self.rootWindow.voiceComboBox;
        self.speechPlayButton = self.rootWindow.speechPlayButton;
        self.speechStopButton = self.rootWindow.speechStopButton;
        self.speechInsertButton = self.rootWindow.speechInsertButton;
        self.speechTextBox = self.rootWindow.speechTextBox;
            
        # Which Cepstral voices are licensed on this machine?
        
        availableVoices = self.textToSpeechProvider.availableVoices();
        try:
            self.cepstralVoicesThisMachine = availableVoices['cepstral'];
        except KeyError:
            self.cepstralVoicesThisMachine = [];
            
        try:    
            self.festivalVoicesThisMachine = availableVoices['festival'];
        except KeyError:
            self.festivalVoicesThisMachine = [];
        
        allVoices = []
        for voice in self.cepstralVoicesThisMachine:
            allVoices.append(voice);
        for voice in self.festivalVoicesThisMachine:
            if voice == 'voice_kal_diphone':
                voice = 'Computerish';
            allVoices.append(voice);

        allVoices.sort();
        for voice in allVoices:
            self.voiceComboBox.addItem(voice);
            
        # No play stop time scheduled:
#        self.scheduledPlayStopTime = None;
        
        # Load QtCreator's XML UI files:
        # Make QtCreator generated UIs children if this instance:
#        self.loadUIs();
#
#        # The tabbed pannel for the command sets:
#        self.toolStack =  QTabWidget()
#        self.toolStack.addTab(self.pickSongWidget, "Pick a song");
#        self.toolStack.addTab(self.tapBeatWidget, "Tap the beat");
#        self.toolStack.addTab(self.beatChooserWidget, "Use the beat");
#        self.toolStack.addTab(self.insertMultiMediaControlsWidget, "Insert songs/sounds to program");
#        self.toolStack.addTab(self.speechWidget, "Robot says...");
#        
#        self.tapeRecWidget.setParent(self.tapBeatWidget.recorderContainerWidget);
#
#        layout =  QVBoxLayout()
#        layout.addWidget(self.toolStack);
#        self.setLayout(layout)
#        self.setGeometry(Tapper.WINDOW_SIZE);
        
        # Make simple names for the widgets we care about:
                
#        # Pick-a-song panel:
#        self.pickSongList = self.pickSongWidget.pickSongList; # Combobox
#        self.pickSongPlayExcerptButton = self.pickSongWidget.shortPlayButton
#        self.pickSongStopExcerptButton = self.pickSongWidget.shortPlayStopButton
#        self.pickSongSongSampleLenSpinBox = self.pickSongWidget.songSampleLength
#        
#        # Tap-the-beat panel:
#        self.tapButton        = self.tapBeatWidget.tapButton;
#        self.tapBeatPeriodLCD = self.tapBeatWidget.beatPeriodLCD;
#        self.tapResetButton   = self.tapBeatWidget.tapResetButton;
#        
#        # Beat chooser panel:
#        self.beatChooserSongName = self.beatChooserWidget.songName;
#        self.beatChooserOneBeatButton = self.beatChooserWidget.oneBeat;
#        self.beatChooserHalfBeatButton = self.beatChooserWidget.halfBeat;
#        self.beatChooserQuarterBeatButton = self.beatChooserWidget.quarterBeat;
#        self.beatChooserEighthBeatButton = self.beatChooserWidget.eighthBeat;
#        self.beatChooserTotalBeatsLabel = self.beatChooserWidget.totalBeatsNumLabel;
#        self.beatChooserTotalDurationLabel = self.beatChooserWidget.totalDurationNumLabel;
#        self.beatChooserInsertTimeButton = self.beatChooserWidget.insertTimeButton;
#        self.beatChooserClearButton = self.beatChooserWidget.clearButton;
#        self.beatChooserMotionDurationTextField = self.beatChooserWidget.insertTimeButton; 


#        # Tape recorder embedded in the Tap-the-beat panel:        
#        self.playButton = self.tapeRecWidget.playButton;
#        self.rewindButton = self.tapeRecWidget.rewindButton;
#        self.backALittleButton =  self.tapeRecWidget.littleBitLeftButton;
#        self.forwardALittleButton =  self.tapeRecWidget.littleBitRightButton;
#        self.stopButton = self.tapeRecWidget.stopButton;
#        self.incrementalMoveTimeSpinBox = self.tapeRecWidget.smallMoveTimeSpinBox;
#        self.songPositionSpinBox = self.tapeRecWidget.songPositionSpinbox;
              
#        # Song/Sound insert panel:
#        #*****
#        self.insertCmdSongToInsertLabel = self.insertMultiMediaControlsWidget.songLabel;
#        #*****        
#        self.insertCmdSongToInsert = self.insertMultiMediaControlsWidget.songSelectComboBox;
#        self.insertCmdStartSongButton = self.insertMultiMediaControlsWidget.songInsertSongButton;
#        self.insertCmdStartPos = self.insertMultiMediaControlsWidget.songStartPosSpinBox;
#        self.insertCmdSongRepeats = self.insertMultiMediaControlsWidget.songRepeatsSpinBox;
#        self.insertCmdPlayDuration = self.insertMultiMediaControlsWidget.songPlayForSpinBox;
#        self.insertCmdWaitTillSongDone = self.insertMultiMediaControlsWidget.waitTillSongDoneCheckBox;
#        
#        self.insertCmdPauseSongButton = self.insertMultiMediaControlsWidget.songInsertPauseButton;
#        self.insertCmdUnpauseSongButton = self.insertMultiMediaControlsWidget.songInsertUnpauseButton;
#        self.insertCmdStopSongButton = self.insertMultiMediaControlsWidget.songInsertStopSongButton;
#        
#        # Various controls, like wait():
#        self.insertCmdWaitButton = self.insertMultiMediaControlsWidget.insertWaitButton;
#        self.insertCmdWaitValue  = self.insertMultiMediaControlsWidget.insertWaitValueSpinBox;
#        
#        self.insertCmdSoundToInsert = self.insertMultiMediaControlsWidget.soundSelectComboBox;
#        self.insertCmdStartSoundButton = self.insertMultiMediaControlsWidget.soundInsertSoundButton;
#        self.insertCmdSoundRepeats = self.insertMultiMediaControlsWidget.soundRepeatsSpinBox;
#        self.insertCmdWaitTillSoundDone = self.insertMultiMediaControlsWidget.waitTillSoundDoneCheckBox;
#        self.insertCmdSoundTestPlayButton = self.insertMultiMediaControlsWidget.soundTestPlayButton;
#        self.insertCmdSoundTestStopButton = self.insertMultiMediaControlsWidget.soundTestStopButton;
#        
#        self.insertCmdPauseSoundButton = self.insertMultiMediaControlsWidget.soundInsertPauseButton;
#        self.insertCmdUnpauseSoundButton = self.insertMultiMediaControlsWidget.soundInsertUnpauseButton;
#        self.insertCmdStopSoundButton = self.insertMultiMediaControlsWidget.soundInsertStopButton;
#        
        
        # Robot speech panel:
#        self.speechDavidVoice    = self.davidVoiceRadioButton;
#        self.speechAmyVoice      = self.amyVoiceRadioButton;
#        self.speechShoutVoice    = self.shoutVoiceRadioButton;
#        self.speechWhisperVoice  = self.whisperVoiceRadioButton;
#        self.speechComputerVoice = self.computerVoiceRadioButton;
        
        # Activate the default voice radio button:
#        self.speechDavidVoice.toggle();

#        # Ensure that the flaky icons appear on the tape recorder buttons:
#        smallTimeBackIcon = QIcon(os.path.join(IMAGE_DIRECTORY, "littleBitBack.png"));
#        smallTimeForwIcon = QIcon(os.path.join(IMAGE_DIRECTORY, "littleBitForward.png"));        
#        playIcon          = QIcon(os.path.join(IMAGE_DIRECTORY, "play.png"));
#        stopIcon          = QIcon(os.path.join(IMAGE_DIRECTORY, "stop.png"));
#        rewindIcon        = QIcon(os.path.join(IMAGE_DIRECTORY, "rewind.png"));
        
#        # Pick-a-song tab:
#        self.pickSongPlayExcerptButton.setIcon(playIcon);
#        self.pickSongStopExcerptButton.setIcon(stopIcon);
#        
#        # Tape recorder on tap-the-beat tab:
#        self.playButton.setIcon(playIcon);
#        self.stopButton.setIcon(stopIcon);
#        self.backALittleButton.setIcon(smallTimeBackIcon);
#        self.forwardALittleButton.setIcon(smallTimeForwIcon);
#        self.rewindButton.setIcon(rewindIcon);
#        
#        # Play briefly in insert sound tab:
#        self.insertCmdSoundTestPlayButton.setIcon(playIcon);
#        self.insertCmdSoundTestStopButton.setIcon(stopIcon);

#        # Sound play in robot says tab:
#        self.speechPlayButton.setIcon(playIcon);
#        self.speechStopButton.setIcon(stopIcon);

        self.connectWidgets()
        
#        # Populate the song lists from the file system:
#        self.populateSongList(self.pickSongList)
#        self.populateSongList(self.beatChooserSongName);
#        self.populateSongList(self.insertCmdSongToInsert);
#        
#        # Populate the sound list:
#        self.populateSoundList(self.insertCmdSoundToInsert);
#    
#        # Prepare the play and pause icons for the tape recorder button:
#        self.playIcon  = QIcon(os.path.join(IMAGE_DIRECTORY, "play.png"));
#        self.pauseIcon = QIcon(os.path.join(IMAGE_DIRECTORY, "pause.png"));
#        self.currentPlayPauseIconState = PlayPauseIconState.PAUSE_ICON_SHOWING;

        #self.initStyling();
        

    def initStyling(self):
        '''
        Set CSS style sheets for various UI elements.
        '''
        
        # Overall application background:
        #appBGColor = QColor(13,7,133); 
        

        steelBlueDarkBGColor = QColor(114, 153, 165);      # Dark background (steel blue)
        steelBlueLightBGColor = QColor(160, 182, 191);     # Light background (steel blue + 1 tint levels)
        steelBlueDarkBGInputsColor = QColor(140,169,179);  # Input element backgrounds used with Dark Background
        
        #appBGColor = steelBlueDarkBGColor;
        appBGColor = steelBlueLightBGColor;

        # Tab background color definition:
        tabBGColor = steelBlueDarkBGColor;
        tabBarBGColor = QColor(178,160,174);  # Rose (related to steel blue)


        controlInsertGroupBGColor = steelBlueDarkBGColor;
        songInsertGroupBoxBGColor = steelBlueLightBGColor;
        soundInsertGroupBoxBGColor = controlInsertGroupBGColor  # steel blue

        # Button color definitions:
        #buttonBGOnDark = QColor(176,220,245); # Light blue
        buttonBGOnDark = QColor(140,169,179); # One tint level lighter than steel blue
        recorderButtonDisabledBGColor = QColor(187,200,208); # Gray-blue
        buttonTextColor = QColor(0,0,0);     # Black

        # Control buttons:
        
        controlButtonBGColor = buttonBGOnDark;
        controlButtonTextColor = buttonTextColor;
        controlSpinBoxBGColor = controlButtonBGColor; 
        controlSpinBoxTextColor = controlInsertGroupBGColor;
        
        comboBoxBGColor = buttonBGOnDark;
        comboBoxTextColor = buttonTextColor;
        
        
        # Application style:
        appStylesheet =\
        'QMainWindow {background-color: ' + appBGColor.name() +\
        '}' +\
        'QWidget {background-color: ' + appBGColor.name() +\
        '}' +\
        'QDoubleSpinBox {background-color: ' + appBGColor.name() +\
        '}';

        # Overall application stylesheet:
        tabWidgetStylesheet =\
            'QTabWidget {background-color: ' + tabBGColor.name() +\
            '}';
        
        # Tabs on the tab widget:
        tabBarStylesheet =\
            'QTabBar {background-color: ' + tabBarBGColor.name() +\
            '}';
        
        controlButtonStylesheet =\
            'QPushButton {background-color: ' + controlButtonBGColor.name() +\
            '; color: ' + controlButtonTextColor.name() +\
            '}';
        
        # Button stylesheets:    
        recorderButtonStylesheet =\
            'QPushButton {background-color: ' + buttonBGOnDark.name() +\
            '; color: ' + buttonTextColor.name() +\
            '}';
            
        recorderButtonDisabledStylesheet =\
            'QPushButton {background-color: ' + recorderButtonDisabledBGColor.name() +\
            '; color: ' + buttonTextColor.name() +\
            '}';
        
        comboBoxStylesheet =\
            'QComboBox {background-color: ' + comboBoxBGColor.name() +\
            '; color: ' + comboBoxTextColor.name() +\
            '}';
        
        spinboxStylesheet =\
            'font-size: ' + str(Tapper.RADIO_BUTTON_LABEL_FONT_SIZE) + 'px' +\
            '; color: ' + buttonTextColor.name() +\
            '; background-color: ' + buttonBGOnDark.name();
        
        tapSpinboxStylesheet =\
            'font-size: ' + str(Tapper.TAP_LABEL_FONT_SIZE) + 'px' +\
            '; color: ' + buttonTextColor.name() +\
            '; background-color: ' + buttonBGOnDark.name();
        

        controlInsertGroupBoxStylesheet =\
            'QGroupBox {background-color: ' + soundInsertGroupBoxBGColor.name() +\
            '}';
            
        songInsertGroupBoxStylesheet =\
            'QGroupBox {background-color: ' + songInsertGroupBoxBGColor.name() +\
            '}';

        soundInsertGroupBoxStylesheet =\
            'QGroupBox {background-color: ' + soundInsertGroupBoxBGColor.name() +\
            '}';


        robotSpeaksStyleSheet =\
            'QTextEdit {background-color: ' + buttonBGOnDark.name() +\
            '}';

        # Basic color:
        self.setStyleSheet(appStylesheet);
        
        self.toolStack.tabBar().setStyleSheet(tabBarStylesheet);
        self.toolStack.setStyleSheet(tabWidgetStylesheet);
        
        # Pick a song tab:
        self.pickSongPlayExcerptButton.setStyleSheet(recorderButtonStylesheet);
        self.pickSongStopExcerptButton.setStyleSheet(recorderButtonStylesheet);
        self.pickSongList.setStyleSheet(comboBoxStylesheet);
        self.pickSongSongSampleLenSpinBox.setStyleSheet(spinboxStylesheet)
        
        # Tapping tab:
        self.tapButton.setStyleSheet(recorderButtonStylesheet);
        self.tapResetButton.setStyleSheet(recorderButtonStylesheet);
        self.tapBeatPeriodLCD.setStyleSheet(tapSpinboxStylesheet);
        
        # Tape recorder style:
        self.playButton.setStyleSheet(recorderButtonStylesheet);
        self.rewindButton.setStyleSheet(recorderButtonStylesheet);
        self.backALittleButton.setStyleSheet(recorderButtonStylesheet);
        self.forwardALittleButton.setStyleSheet(recorderButtonStylesheet);
        self.stopButton.setStyleSheet(recorderButtonStylesheet);
        self.incrementalMoveTimeSpinBox.setStyleSheet(spinboxStylesheet);
        self.songPositionSpinBox.setStyleSheet(spinboxStylesheet);
        
        # Beat chooser style:
        self.beatChooserSongName.setStyleSheet(comboBoxStylesheet);
        self.beatChooserOneBeatButton.setStyleSheet(recorderButtonStylesheet);
        self.beatChooserHalfBeatButton.setStyleSheet(recorderButtonStylesheet);
        self.beatChooserQuarterBeatButton.setStyleSheet(recorderButtonStylesheet);
        self.beatChooserEighthBeatButton.setStyleSheet(recorderButtonStylesheet);
        self.beatChooserClearButton.setStyleSheet(recorderButtonStylesheet);
        self.beatChooserInsertTimeButton.setStyleSheet(recorderButtonStylesheet);


        
#        self.insertMultiMediaControlsWidget.flowControlGroupBox.setStyleSheet(controlInsertGroupBoxStylesheet);
#        #self.insertMultiMediaControlsWidget.songInsertControlsGroupBox.setStyleSheet(songInsertGroupBoxStylesheet);
#        #self.insertMultiMediaControlsWidget.songInsertControlsGroupBox.setStyleSheet(controlInsertGroupBoxStylesheet);
#        self.insertMultiMediaControlsWidget.soundInsertSoundGroupBox.setStyleSheet(soundInsertGroupBoxStylesheet);
        
        self.insertCmdWaitButton.setStyleSheet(controlButtonStylesheet);
        self.insertCmdWaitValue.setStyleSheet(spinboxStylesheet);
        
        #****
#        testSheet =\
#            'QLabel {background-color: ' + songInsertGroupBoxBGColor.name() +\
#            '}';
#            
#        self.insertCmdSongToInsertLabel.setStyleSheet(testSheet);
        #****
        self.insertCmdSongToInsert.setStyleSheet(comboBoxStylesheet);
        self.insertCmdSoundTestPlayButton.setStyleSheet(recorderButtonStylesheet);
        self.insertCmdSoundTestStopButton.setStyleSheet(recorderButtonStylesheet);
        self.insertCmdStartSongButton.setStyleSheet(recorderButtonStylesheet);
        self.insertCmdPauseSongButton.setStyleSheet(recorderButtonStylesheet);
        self.insertCmdUnpauseSongButton.setStyleSheet(recorderButtonStylesheet);
        self.insertCmdStopSongButton.setStyleSheet(recorderButtonStylesheet);
        self.insertCmdStartPos.setStyleSheet(spinboxStylesheet);
        self.insertCmdSongRepeats.setStyleSheet(spinboxStylesheet);
        self.insertCmdPlayDuration.setStyleSheet(spinboxStylesheet);
        
        self.insertCmdSoundToInsert.setStyleSheet(comboBoxStylesheet);
        self.insertCmdStartSoundButton.setStyleSheet(recorderButtonStylesheet);
        self.insertCmdPauseSoundButton.setStyleSheet(recorderButtonStylesheet);
        self.insertCmdUnpauseSoundButton.setStyleSheet(recorderButtonStylesheet);
        self.insertCmdStopSoundButton.setStyleSheet(recorderButtonStylesheet);        
        self.insertCmdSoundRepeats.setStyleSheet(spinboxStylesheet);
        
        # Speech:
        self.speechTextBox.setStyleSheet(robotSpeaksStyleSheet);
        self.speechPlayButton.setStyleSheet(recorderButtonStylesheet);
        self.speechStopButton.setStyleSheet(recorderButtonStylesheet);
        self.speechInsertButton.setStyleSheet(recorderButtonStylesheet);

    def connectWidgets(self):
        '''
        Link buttons to methods that will act on button pushes.
        '''
        
        # Tab switch signal:
#        self.toolStack.currentChanged.connect(self.switchedTab);
#        
#        # Pick-a-song panel:
#        self.pickSongPlayExcerptButton.clicked.connect(self.playSongExcerptAction);
#        self.pickSongStopExcerptButton.clicked.connect(self.stopSongExcerptAction);
#
#        # Tapping the beat:
#        self.tapButton.clicked.connect(self.tapButtonAction);
#        self.tapResetButton.clicked.connect(self.tapResetAction);
#        self.tapBeatPeriodLCD.valueChanged.connect(self.beatLCDChanged);
        
        # Tape recorder:
#        self.playButton.clicked.connect(self.playAction);
#        self.rewindButton.clicked.connect(self.rewindAction);
#        self.backALittleButton.clicked.connect(self.backALittleAction);
#        self.forwardALittleButton.clicked.connect(self.forwardALittleAction);
#        self.stopButton.clicked.connect(self.stopAction);
#        self.songPositionSpinBox.valueChanged.connect(self.setPlayheadFromSpinBoxAction);

#        # Make time delays:
#        self.beatChooserSongName.currentIndexChanged.connect(self.beatNewSongAction);
#        self.beatChooserOneBeatButton.clicked.connect(partial(self.beatButtonAction, 1.0));
#        self.beatChooserHalfBeatButton.clicked.connect(partial(self.beatButtonAction, 0.5));
#        self.beatChooserQuarterBeatButton.clicked.connect(partial(self.beatButtonAction, 0.25));
#        self.beatChooserEighthBeatButton.clicked.connect(partial(self.beatButtonAction, 0.125));
#        self.beatChooserClearButton.clicked.connect(self.beatClearAction);
#        self.beatChooserInsertTimeButton.clicked.connect(self.beatInsertTimeAction);
#        
#        # Inserting multimedia actions into the program:
#        self.insertCmdStartSongButton.clicked.connect(self.insertCmdStartSongButtonAction)        
#        self.insertCmdPauseSongButton.clicked.connect(self.insertCmdPauseSongButtonAction);
#        self.insertCmdUnpauseSongButton.clicked.connect(self.insertCmdUnpauseSongButtonAction);
#        self.insertCmdStopSongButton.clicked.connect(self.insertCmdStopSongButtonAction);
#    
#        self.insertCmdWaitButton.clicked.connect(self.insertCmdWaitButtonAction);
#        
#        self.insertCmdStartSoundButton.clicked.connect(self.insertCmdStartSoundButtonAction)
#        self.insertCmdPauseSoundButton.clicked.connect(self.insertCmdPauseSoundButtonAction);
#        self.insertCmdUnpauseSoundButton.clicked.connect(self.insertCmdUnpauseSoundButtonAction);
#        self.insertCmdStopSoundButton.clicked.connect(self.insertCmdStopSoundButtonAction);
#        
#        self.insertCmdSoundTestPlayButton.clicked.connect(self.insertCmdTestPlaySoundLocallyAction);
#        self.insertCmdSoundTestStopButton.clicked.connect(self.insertCmdStopTestPlaySoundLocallyAction);
#        
        # Text to speech:
        
        self.speechPlayButton.clicked.connect(self.sayAction);
        self.speechStopButton.clicked.connect(self.stopSayingAction);
        self.speechInsertButton.clicked.connect(self.insertSaySomethingAction);
        
#        # Blink pause icon on/off during paused playback:
#        self.commChannel.pauseBlinkSignal.connect(self.blinkPauseButtonIcon);
#        # Update playhead pos counter during playback:
#        self.commChannel.playCounterUpdateSignal.connect(self.updatePlayheadCounter)

    def closeEvent(self, eventObj):
        '''
        Hanlde window close events.
        @param eventObj: event loop generated close event information object.
        @type eventObj: ?
        '''
        try:
#            self.maintenanceThread.stop();
            eventObj.accept();
        except:
            pass;
        
    def switchedTab(self, newTabIndex):
        '''
        Called automatically called when user switches tabs on the Tapper window.
        @param newTabIndex: Zero based tab index.
        @type newTabIndex: int
        '''
        #print str(self.geometry());
        if newTabIndex == Tapper.TAB_INDEX_PICK_SONG:
            self.populateSongList(self.pickSongList);
    
    def populateSongList(self, comboBoxToPopulate, songNameList=None):
        '''
        Populate the pull-down list of available songs. The song names are
        determined by the file names of the sounds/music subdirectory of the 
        SpeakEasy package.
        @param comboBoxToPopulate: Qt combo box instance that will hold the song names.
        @type comboBoxToPopulate: QComboBox
        @param songNameList: list of available songs. Default is to explore the package's 
                             sounds/music subdirectory. 
        @type songNameList: [string]
        '''
        if songNameList is None:
            # Get songs from standard song directory
            filenames = os.listdir(self.SONG_DIRECTORY);
            self.allSongs = {};
            for filename in filenames:
                # From /foo/bar/blue.txt, get (blue, .txt):
                (fileBaseName, extension) = os.path.splitext(filename);
                try:
                    self.musicPlayer.formatSupported(extension);
                    # Map file name without extension (i.e. song name) to full path: 
                    self.allSongs[fileBaseName] = os.path.join(self.SONG_DIRECTORY,filename);
                except ValueError:
                    continue;
            if (len(self.allSongs) == 0):
                comboBoxToPopulate.addItem(Tapper.EMPTY_SONG_LIST_TEXT);
            else:
                comboBoxToPopulate.addItems(self.allSongs.keys());
        else:
            comboBoxToPopulate.addItems(songNameList); 

    def populateSoundList(self, comboBoxToPopulate, soundNameList=None):
        '''
        Populate the pull-down list of available sounds. The soound names are
        determined by the file names of the sounds subdirectory of the 
        SpeakEasy package.
        @param comboBoxToPopulate: Qt combo box instance that will hold the song names.
        @type comboBoxToPopulate: QComboBox
        @param soundNameList: list of available sounds. Default is to explore the package's 
                             sounds subdirectory. 
        @type soundNameList: [string]
        '''
        if soundNameList is None:
            # Get songs from standard song directory
            filenames = os.listdir(self.SOUND_DIRECTORY);
            self.allSounds = {};
            for filename in filenames:
                # From /foo/bar/blue.txt, get (blue, .txt):
                (fileBaseName, extension) = os.path.splitext(filename);
                try:
                    self.soundPlayer.formatSupported(extension);
                    # Map file name without extension (i.e. song name) to full path: 
                    self.allSounds[fileBaseName] = os.path.join(self.SOUND_DIRECTORY,filename);
                except ValueError:
                    continue;
            if (len(self.allSounds) == 0):
                comboBoxToPopulate.addItem(Tapper.EMPTY_SOUND_LIST_TEXT);
            else:
                comboBoxToPopulate.addItems(self.allSounds.keys());
        else:
            comboBoxToPopulate.addItems(soundNameList); 


    def setPlayPauseButtonIcon(self, icon):
        '''
        Replace the tape recorder play/pause button's icon. Used to 
        flash between play and pause if the recorder is in pause mode.
        @param icon: Image to show on play/pause button.
        @type icon: QIcon
        '''
        self.playButton.setIcon(icon);

    def schedulePlayStop(self, secondsToPlay):
        '''
        Schedule a timer that fires when a song is to be stopped
        programmatically. Used for pose synchronization. 
        @param secondsToPlay: Number of seconds to continue playing.
        @type secondsToPlay: float
        '''
        if secondsToPlay is None or secondsToPlay == -1 or secondsToPlay == 0.0:
            return;
        try:
            self.scheduledPlayStopTime = time.time() + float(secondsToPlay);
        except (TypeError, ValueError):
            raise ValueError("Seconds to play must be a float, or value convertible to a float. Instead: " + str(secondsToPlay));

    def cancelScheduledPlayStop(self):
        '''
        Abandon a previously scheduled stop play action.
        '''
        self.scheduledPlayStopTime = None;

    def rewindAction(self):
        '''
        Tape recorder button: Rewind to beginning.
        '''
        self.musicPlayer.stop();
        self.setPlayPauseButtonIcon(self.playIcon);
        currentSongName = self.getCurrentSongName();
        if currentSongName is None:
            return;
        self.musicPlayer.play(currentSongName)
        #******self.setPlayPauseButtonIcon(self.pauseIcon);
        
    def backALittleAction(self):
        '''
        Tape recorder action to rewind a few, fixed seconds of a song.
        '''
        if self.musicPlayer.playStatus == PlayStatus.PLAYING:        
            self.musicPlayer.setPlayhead(-1 * self.getIncrementalMoveTime(), TimeReference.RELATIVE);
        
    def forwardALittleAction(self):
        '''
        Tape recorder action to wind forward a few, fixed seconds of a song.
        '''
        if self.musicPlayer.playStatus == PlayStatus.PLAYING:
            self.musicPlayer.setPlayhead(self.getIncrementalMoveTime(), TimeReference.RELATIVE);
        
    def playAction(self):
        '''
        Tape recorder action: Being playback.
        '''
        # The play button shares with the pause function.
        # If we are currently playing, this button press indicates
        # that the user wants to pause.
        playStatus = self.musicPlayer.getPlayStatus();
        if playStatus == PlayStatus.PAUSED:
            self.musicPlayer.unpause();
            #****self.setPlayPauseButtonIcon(self.pauseIcon);
            return;
        elif playStatus == PlayStatus.PLAYING:
            self.musicPlayer.pause()
            self.currentPlayPauseIconState = PlayPauseIconState.PAUSE_ICON_SHOWING;
            #****self.setPlayPauseButtonIcon(self.playIcon);
            return;
        else:
            # Play is stopped. Just play the song from the start again: 
            fullFilename = self.getCurrentSongName();
            self.musicPlayer.play(fullFilename);
            self.setPlayPauseButtonIcon(self.pauseIcon);
        
    def stopAction(self):
        '''
        Tape recorder stop button action.
        '''
        self.musicPlayer.stop();
        self.setPlayPauseButtonIcon(self.playIcon);

    def setPlayheadFromSpinBoxAction(self, newVal):
        '''
        Set the tape recorder playhead to a particular time. Only works
        with .ogg files.
        @param newVal: Fractional playehead time setting.
        @type newVal: {float | string}
        '''
        # Interrupt from user having changed the playhead spinbox:
        if not isinstance(newVal, float):
            return;
        self.musicPlayer.setPlayhead(newVal, TimeReference.ABSOLUTE);

    def playSongExcerptAction(self):
        '''
        Play a song snippet.
        '''
        
        if self.getCurrentSongName() is None:
            return
        sampleLen = self.pickSongSongSampleLenSpinBox.value();
        
        # Find a random spot in the song, and play for sampleLen 
        # seconds. Make sure that if the song is very short, move
        # start time closer to the start of the song:

        # Avg song is 3 minutes (180 seconds). Pick an upper
        # time bound below that, but not too low, so that random
        # snippets will be played from all over the song:
        highestStartTime = 170; # sec
        random.seed();        
        playedSample = False;
        while not playedSample:
            sampleStart = random.randint(0,highestStartTime); 
            self.musicPlayer.play(self.getCurrentSongName(), startTime=float(sampleStart), blockTillDone=False);
            if self.musicPlayer.getPlayStatus() != PlayStatus.PLAYING:
                highestStartTime = int(highestStartTime / 2);
                continue;
            else:
                # Schedule for playback to stop after sampleLen seconds.
                # This action will be taken by the maintenance thread:
                self.schedulePlayStop(sampleLen);
                return;

    def stopSongExcerptAction(self):
        '''
        Stop song snippet playing in pick-a-song panel.
        '''
        self.musicPlayer.stop();

    def tapButtonAction(self):
        '''
        Action bound to Tap button. Invites the user tp click the mouse
        Accumulates BPM (Beats per minutes) time counts.
        '''
        
        # If more than 15 seconds since the previous tap,
        # or since initialization, start over:
        if (time.time() - self.mostRecentTapTime) > 15:
            self.resetTap();
        
        self.numTaps += 1;
        self.mostRecentTapTime = time.time();
        self.currentBeatPerSecs = float(self.numTaps) / (self.mostRecentTapTime - self.firstTapTime);
        # Convert to beats per minute:
        self.currentBeat       = self.currentBeatPerSecs * 60.0;
        #self.songBeats[self.getCurrentSongName()] = self.currentBeat;
        self.tapBeatPeriodLCD.setValue(self.currentBeat); 
        
    def beatLCDChanged(self, newVal):
        '''
        Signal receptor that the text box containing song beat was changed.
        @param newVal:
        @type newVal:
        '''
        self.songBeats[self.getCurrentSongName()] = newVal;

    def tapResetAction (self):
        '''
        Bound to button that requests setting the Beats per Minute information to zero.
        '''
        self.resetTap();

    def resetTap(self):
        '''
        Machinery to zero out BPM.
        '''
        self.firstTapTime = time.time();
        self.mostRecentTapTime = self.firstTapTime;
        self.currentBeat = 0.0; 
        self.numTaps = 0;
        self.tapBeatPeriodLCD.setValue(0.0);

    # Song insertion:

    def insertCmdStartSongButtonAction(self):
        '''
        Insert a start song command into the pose list.
        '''
        self.commChannel.insertStartSongSignal.emit(self.insertCmdSongToInsert.currentText(),
                                                    self.insertCmdStartPos.value(),
                                                    self.insertCmdSongRepeats.value(),
                                                    self.insertCmdPlayDuration.value(),
                                                    self.insertCmdWaitTillSongDone.isChecked());

    def insertCmdPauseSongButtonAction(self):
        '''
        Insert a Pause song command into the pose list.
        '''
        self.commChannel.insertPauseSongSignal.emit();
        
    def insertCmdUnpauseSongButtonAction(self):
        '''
        Insert an unpause command into the pose list.
        '''
        self.commChannel.insertUnPauseSongSignal.emit();
        
    def insertCmdStopSongButtonAction(self):
        '''
        Insert a stop command into the pose list.
        '''
        self.commChannel.insertStopSongSignal.emit();
    
    # Various controls, like wait():
    
    def insertCmdWaitButtonAction(self):
        '''
        Insert command in pose list that will wait for a fixed number
        of fractional seconds (see self.insertCmdWaitValue.value()).
        '''
        self.commChannel.insertWaitSignal.emit(self.insertCmdWaitValue.value());
        
    # Sound effect insertion:
    def insertCmdStartSoundButtonAction(self):
        '''
        Insert command in pose list that will start a sound effect.  
        '''
        self.commChannel.insertStartSoundSignal.emit(self.insertCmdSoundToInsert.currentText(),
                                                     self.insertCmdSoundRepeats.value(),
                                                     self.insertCmdWaitTillSoundDone.isChecked());
    
    def insertCmdPauseSoundButtonAction(self):
        '''
        Insert command in pose list that will pause a sound effect.  
        '''
        self.commChannel.insertPauseSoundSignal.emit();
        
    def insertCmdUnpauseSoundButtonAction(self):
        '''
        Insert command in pose list that will unpause a sound effect.  
        '''
        self.commChannel.insertUnPauseSoundSignal.emit();
        
    def insertCmdStopSoundButtonAction(self):
        '''
        Insert command in pose list that will stop a sound effect.  
        '''
        self.commChannel.insertStopSoundSignal.emit();

    def insertCmdTestPlaySoundLocallyAction(self):
        '''
        Locally play sound ahead of inserting it.
        '''
        self.soundPlayer.play(self.getCurrentSoundName(), blockTillDone=False);
    
    
    def insertCmdStopTestPlaySoundLocallyAction(self):
        '''
        Stop locally playing sound ahead of inserting it.
        '''
        self.soundPlayer.stop();
        
    def sayAction(self):
        '''
        Bound to speech button in Tapper UI. Reads various text-to-speech
        parameters from the Tapper window: the voice and tts engine to use.
        Then inserts a corresponding start speech command into the Puppet pose list.
        '''
        
        whatToSay = self.speechTextBox.toPlainText();
        if len(whatToSay) == 0:
            self.dialogService.showErrorMsg("Please type something to say into the text box.");
            return;
        voiceName = self.getSelectedVoiceName();
        if voiceName in self.cepstralVoicesThisMachine:
            ttsEngine = 'cepstral'
        elif voiceName == 'Computerish':
            voiceName = 'voice_kal_diphone';
            ttsEngine = 'festival';
        else:
            ttsEngine = None;
            
        # Speak the text, using the default t2s engine, and unblocked operation:
        self.textToSpeechProvider.say(whatToSay, voiceName=voiceName, t2sEngineName=ttsEngine, blockTillDone=False);

    def stopSayingAction(self):
        '''
        Insert command in pose list that will stop text-to-speech.  
        '''
        self.textToSpeechProvider.stop();
        
    def insertSaySomethingAction(self):
        '''
        Insert command in pose list that will have the robot speak.  
        '''
        
        whatToSay = self.speechTextBox.toPlainText();
        if len(whatToSay) == 0:
            self.dialogService.showErrorMsg("Please type something to say into the text box.");
            return;
        
        voiceName = self.getSelectedVoiceName();
        # Map the more friendly name 'Computerish' that's used in the 
        # UI to its official name:
        if voiceName == 'Computerish':
            voiceName = 'voice_kal_diphone';
        ttsEngine = self.toolbox.matchVoiceToTTSEngine(voiceName);
        self.commChannel.insertRobotSaysSignal.emit(self.speechTextBox.toPlainText().encode('utf8'), voiceName, ttsEngine);

    def beatClearAction(self):
        '''
        Clear the beat-per-minute display.
        '''
        self.clearMakeDelayTotalBeats();
        self.clearMakeDelayTotalDuration();

    def beatButtonAction(self, beatFraction):
        '''
        Compute pose times based on sound beats. Must be called
        after user tapped or entered beat-per-seconds. Updates Tapper
        beat computation tab.
        @param beatFraction: which fraction of one beat to add to the accumulated
                             beats count.
        @type beatFraction: float
        '''
        currTotalBeats = self.getTotalBeatsDelay();
        currTotalDur   = self.getTotalTimeDelay();
        songName       = self.getBeatChooserSongName();
        try:
            fullBeatDuration = self.songBeats[songName];
        except KeyError:
            # User didn't add a beat;
            # Don't display the whole pathname in the error msg:
            songNameOnly = os.path.basename(songName).split('.')[0];
            self.dialogService.showErrorMsg("Beat for <b>%s</b> was not tapped; or unable to find beat on Internet." % 
                                            songNameOnly)
            return;
        newSumOfBeats = currTotalBeats + beatFraction;
        self.setMakeDelayTotalBeats(newSumOfBeats);
        self.setMakeDelayTotalDuration(float(newSumOfBeats) * (fullBeatDuration / 60.0));
        
    def beatInsertTimeAction(self):
        '''
        Insert a computed beat time obtained from the repective Tapper
        tab into the 'Duration' field of the Puppet main application. 
        '''
        self.commChannel.insertTimeDurationSignal.emit(self.getTotalTimeDelay());     
        
    def beatNewSongAction(self):
        '''
        Signal catch for when current song selection is changed. 
        Clears the beat computations.
        '''
        # New song selected in beat chooser. Clear beats sums:
        self.setTotalBeatsDelay(0.0);
        self.setTotalTimeDelay(0.0);
        
    # -----------------------------------------------------  Signal Handlers -----------------------------
    
    def blinkPauseButtonIcon(self):
        '''
        Handler for signal self.pauseBlinkSignal. The one-second
        maintenance thread generates that signal if the music player
        is currently paused. The method turns the pause button on
        and off, toggling each time.
        '''
        if self.currentPlayPauseIconState == PlayPauseIconState.PLAY_ICON_SHOWING:
            self.playButton.setIcon(self.pauseIcon);
            self.currentPlayPauseIconState = PlayPauseIconState.PAUSE_ICON_SHOWING;
        else:
            self.playButton.setIcon(self.playIcon);
            self.currentPlayPauseIconState = PlayPauseIconState.PLAY_ICON_SHOWING;
    
    def updatePlayheadCounter(self):
        '''
        Hander for signal self.playCounterUpdate. The one-second
        maintenance thread generates that signal if the music player
        is currently playing.
        '''
        playheadPos = self.musicPlayer.getPlayheadPosition();
        self.songPositionSpinBox.setValue(playheadPos);

#------------------------------------------------------ Private Utilities --------------------------

    def getCurrentSongName(self, songBasename=None):
        '''
        Without a songBasename, returns the full filename of the song
        listed in the Pick-a-song's pull-down list. If songBaseName is 
        provided, it is expected to be a song that is registered
        in  the self.allSongs dict. 
        @param songBasename: If provided, name of a readable song file basename.
        @type songBasename: string
        @rtype: {string | None}
        '''
        if songBasename is None:
            return self.getCurrentFileName(self.pickSongList, SoundType.SONG);
        else:
            try:
                return self.allSongs[songBasename];
            except KeyError:
                return None;
    
    def getCurrentSoundName(self):
        return self.getCurrentFileName(self.insertCmdSoundToInsert, SoundType.SOUND_EFFECT);

    def getFileNameFromAudioName(self, audioName, soundType):
        '''
        Return full pathname for a given song or sound name
        @param audioName: Name of a song or sound. Ex: "cottonFields", "moo"
        @type audioName: string
        @param soundType: indicator whether given combobox contains a song or sound file.
        @type soundType: SoundType
        @return: full pathname of the given song or sound.
        @rtype: string
        '''
        if soundType == SoundType.SONG:
            try:
                currentSong = self.allSongs[audioName];
            except KeyError:
                return None;
            return currentSong;
        else:
            try:
                currentSound = self.allSounds[audioName];
            except KeyError:
                return None;
            return currentSound;
        

    def getCurrentFileName(self, comboboxObj, soundType):
        '''
        Return full pathname for the song or sound name in the given pick-a-song's or
        pick-a-sound's combobox:
        @param comboboxObj: Instance of QT4 combobox, which contains the name of a song or sound
                            in the music/sound directories, respectively.
        @type comboboxObj: QComboBox
        @param soundType: indicator whether given combobox contains a song or sound file.
        @type soundType: SoundType
        @return: full pathname of the given song or sound.
        @rtype: string
        '''
        currentNameOnComboBox = comboboxObj.currentText();
        if soundType == SoundType.SONG:
            if currentNameOnComboBox == Tapper.EMPTY_SONG_LIST_TEXT:
                return None
            try:
                currentSong = self.allSongs[currentNameOnComboBox];
            except KeyError:
                return None;
            return currentSong;
        else:
            if currentNameOnComboBox == Tapper.EMPTY_SOUND_LIST_TEXT:
                return None
            try:
                currentSound = self.allSounds[currentNameOnComboBox];
            except KeyError:
                return None;
            return currentSound;

    def getIncrementalMoveTime(self):
        return self.incrementalMoveTimeSpinBox.value();

    def getSongPositionIndicator(self):
        return self.songPositionSpinBox.value();
    
    def setSongPositionIndicator(self, newValue):
        if not isinstance(newValue, float):
            raise ValueError("Song position must be a floating point number. Instead it was: " + str(newValue));
        self.songPositionSpinBox.setValue(newValue);
        

    def getSelectedVoiceName(self):
        '''
        Return the voice a user selected in Tapper's text-to-speech tab.
        @return: Speech engine voice that is checked in Tapper window.
        @rtype: string
        '''
        currVoice = self.voiceComboBox.currentText();
        return currVoice;

    def getBeatChooserSongName(self):
        '''
        Provides song name that is currently selected in the Tapper window.
        @returns: Song path.
        @rtype: string
        '''
        return self.getCurrentFileName(self.beatChooserSongName, SoundType.SONG)
        #****return self.beatChooserSongName.currentText();

    def getTotalBeatsDelay(self):
        '''
        Provide current sum of beat calculations.
        @return: Sum of beats so far in the Tapper beat computer tab.
        @rtype: float
        '''
        return float(self.beatChooserTotalBeatsLabel.text());

    def setTotalBeatsDelay(self, totalBeats):
        '''
        Set beat sum calculation in Tapper window.
        (Duplicate of setMakeDelayTotal...)
        @param totalBeats: Fractional beat sum to display in the Tapper beat computation tab.
        @type totalBeats: {float | int | string}|
        '''
        self.beatChooserTotalBeatsLabel.setText(str(totalBeats));
        
    def getTotalTimeDelay(self):
        '''
        Obtain the sum of real time corresponding to beat sum entered by user in 
        Tapper beat computation window.
        @return: Sum of beat fractions summed by user so far.
        @rtype: float
        '''
        return float(self.beatChooserTotalDurationLabel.text());

    def setTotalTimeDelay(self, totalDuration):
        '''
        Set time sum of beat calculation in Tapper beat calculation window.
        (Duplicate of setMakeDelayTotal...)
        @param totalDuration: Sum of real time taken by sum of fractional beats
                              entered by user.
        @type totalDuration: {str | int | float}
        '''
        self.beatChooserTotalDurationLabel.setText(str(totalDuration));

    def clearMakeDelayTotalBeats(self):
        '''
        Set sum of beats in beat calculation tab to zero.
        '''
        self.setMakeDelayTotalBeats(0.0);
        
    def clearMakeDelayTotalDuration(self):        
        '''
        Set sum of real time delay in beat calculation tab to zero.        
        '''
        self.setMakeDelayTotalDuration(0.0);
        
    def setMakeDelayTotalBeats(self, totalBeatsFloat):
        '''
        Set sum of total fractional beats entered by user so far. 
        @param totalBeatsFloat: Sum of beat fractions
        @type totalBeatsFloat: {str | int | float}
        '''
        self.beatChooserTotalBeatsLabel.setText(str(totalBeatsFloat));
        
    def setMakeDelayTotalDuration(self, totalDurationFloat):
        '''
        Set sum of time delay implied by sum of fractional beats.
        @param totalDurationFloat: Real time seconds of delay.
        @type totalDurationFloat: {str | int | float}
        '''
        self.beatChooserTotalDurationLabel.setText(str(totalDurationFloat));
    
    def closeEvent(self, event):
        '''
        Cleanly destroy Tapper window. Called by user clicking X on 
        main application window. Note: Only has an effect if Tapper is
        currently closable. That flag is normally False to prevent users from
        closing the Tapper window without also closing the Puppet main window. 
        @param event: Close event
        @type event: QEvent
        '''
#        if self.closable:
#            self.maintenanceThread.stop();
#            event.accept()
#        else:
#            event.ignore()
        event.accept()

# -------------------------------------------------  One-Second Maintenance Thread ------------------

class OneSecondMaintenance(threading.Thread):
    '''
    Thread for 1-second period chores:
        - When tape recorder is paused, changes play button icon between
          play and pause symbols.
        - For song snippet playing, stops playback after tapper.snippetLength
            seconds.
    '''
    
    def __init__(self, tapper, musicPlayer, pauseBlinkSignal, playCounterSignal):
        '''
        The info the thread needs:
        @param tapper: instance of Tapper (to obtain various instance vars from it).
        @type tapper: Tapper
        @param musicPlayer: Instance of MusicPlayer (to check for play status, and to stop playback)
        @type musicPlayer: MusicPlayer
        @param pauseBlinkSignal: Signal to emit so that the GUI thread will do the play button
                                 icon switching, which only that thread is allowed to do.
        @type pauseBlinkSignal: Signal
        @param playCounterSignal: Signal to emit so that the playhead time counter will be advanced
                                  in the GUI thread.
        @type playCounterSignal: Signal
        '''
        super(OneSecondMaintenance, self).__init__();
        self.stopped = False;
        self.musicPlayer = musicPlayer;
        self.tapper = tapper;
        self.pauseBlinkSignal = pauseBlinkSignal;
        self.playCounterSignal = playCounterSignal;
        
    def run(self):
        while not self.stopped:
            playStatus = self.musicPlayer.getPlayStatus(); 
            if playStatus == PlayStatus.PAUSED:
                self.pauseBlinkSignal.emit();
            elif playStatus == PlayStatus.PLAYING:
                self.playCounterSignal.emit();

            if self.tapper.scheduledPlayStopTime is not None:
                if time.time() >= self.tapper.scheduledPlayStopTime:
                    self.musicPlayer.stop();
                    self.tapper.scheduledPlayStopTime = None;

            time.sleep(1.0);
            
    def stop(self):
        self.stopped = True;
        
#------------------------------------  Testing --------------------------        
        
if __name__ == "__main__":
    
    # No unittest yet.
    print "No unittests implemented yet."
    sys.exit();

#    style = QStyleFactory.create("Cleanlooks");
#    QApplication.setStyle(style);
    app = QApplication(sys.argv);
    rootWindow = QMainWindow();
#        
    toolboxGUI = ToolboxGUI.getInstance(rootWindow)
    toolboxGUI.show()
    # Enter Qt application main loop
    #builderFullUI.show();
    sys.exit(app.exec_());
    
    