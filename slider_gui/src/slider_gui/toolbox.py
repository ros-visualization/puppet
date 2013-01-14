#!/usr/bin/env python


'''
This module is a horrible mess, as is its companion toolbox_gui.py. Both consist
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

from python_qt_binding import QtBindingHelper;
from QtCore import Signal

#from utilities.speakeasy_utils import SpeakeasyUtils; 

from speakeasy.music_player import MusicPlayer;
from speakeasy.music_player import TimeReference;
from speakeasy.music_player import PlayStatus;
from speakeasy.sound_player import SoundPlayer;
from speakeasy.text_to_speech import TextToSpeechProvider;

class SoundType:
    '''
    Enum marking a variable as music, or a sound effect.
    '''
    SONG = 0;
    SOUND_EFFECT = 1;

class Toolbox(object):
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
    
    def __init__(self):
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
        if Toolbox.inst is not None:
            raise RuntimeError("Can only have single instance of Toolbox. Use getInstance() factory method.");
        
        super(Toolbox, self).__init__();
        
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
        
        Toolbox.inst = self;
        self.musicPlayer = MusicPlayer();
        self.soundPlayer = SoundPlayer();
        self.textToSpeechProvider = TextToSpeechProvider();
        
        # Get a dict: 
        #     Cepstral : ['David', 'Anna']
        #     Festival : ['voice_kal_diphone']
        self.allVoices = self.textToSpeechProvider.availableVoices();
        
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
    def getInstance():
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
        if Toolbox.inst is not None:
            return Toolbox.inst;
        return Toolbox();

    def matchVoiceToTTSEngine(self, voice):
        '''
        Given name of a voice, return the test-to-speech engine name
        that provides that voice. Returns None if no engine is found
        for the given voice.
        @param voice: voice to investigate
        @type voice: string
        @return: text-to-speech engine that provides the voice. None if no engine available for that voice.
        '''
        try:
            if voice in self.allVoices['cepstral']:
                return 'cepstral';
        except KeyError:
            # Cepstral voice engine not available.
            pass;
        
        try:
            if voice in self.allVoices['festival']:
                return 'festival'
        except KeyError:
            # Festival voice engine not available.
            pass;

        return None
        
 
if __name__ == "__main__":
    

    toolbox = Toolbox.getInstance()
    
