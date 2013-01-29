7#!/usr/bin/env python

'''
NOTE: A PYTHONPATH problem at a tight time cause src/actions to not be found (another 
such dir was found instead). So this class is temporarily here, not
with the others in ../action. Needs fixing.
'''


import roslib;
roslib.load_manifest('actionlib');
roslib.load_manifest('speakeasy');
import rospy;
import threading;
import copy;

from slider_gui.actions.Action import Action;
from speakeasy.text_to_speech import TextToSpeechProvider;
from speakeasy.markupManagement import MarkupManagement;
from slider_gui.toolbox import Toolbox;


NON_BLOCKING = False;
REMEMBER_SPEAK_INSTANCE = False;
FORGET_SPEAK_INSTANCE = True;

BUSY_CHECK_PERIOD = 0.3; # sec

class Pr2SpeakAction(Action):

    multiSpeechBlockLock = threading.Lock();
    pr2SpeakActionInstances = [];
    
    #----------------------------------
    # stopAllSpeech
    #--------------
    
    @staticmethod
    def stopAllSpeech():
        # Shallow-copy the list of running speech actions 
        # because stopping them one by one will modify
        # the pr2SpeakActionInstances class-level array:
        speakActions = copy.copy(Pr2SpeakAction.pr2SpeakActionInstances); 
        for speakAction in speakActions:
            speakAction.stop(reason=1);
    
    #----------------------------------
    # registerSpeechAct
    #--------------
    
    @staticmethod
    def registerSpeechAct(pr2SpeakActionObj):
        if pr2SpeakActionObj not in Pr2SpeakAction.pr2SpeakActionInstances:
            Pr2SpeakAction.pr2SpeakActionInstances.append(pr2SpeakActionObj);
            
    #----------------------------------
    # unregisterSpeechAct
    #--------------
    
    @staticmethod
    def unregisterSpeechAct(pr2SpeakActionObj):
        try:
            Pr2SpeakAction.pr2SpeakActionInstances.remove(pr2SpeakActionObj);
        except ValueError:
            pass
    
    #----------------------------------
    # Initialization 
    #--------------
    
    #def __init__(self, soundControl=None, utterance=None, voice=None, ttsEngine=None, waitForSpeechDone=False):
    def __init__(self, toolbox=None, utterance='!None!', voice='voice_kal_diphone', ttsEngine='festival', waitForSpeechDone=True):
        super(Pr2SpeakAction, self).__init__();
        
        # Toolbox is None if this instance is created from
        # deserialization.
        if toolbox is None:
            self.toolbox = Toolbox.getInstance();
        else:
            self.toolbox = toolbox;
        
        self.utterance = utterance;
        self.voice = voice;
        self.ttsEngine = ttsEngine;
        self._timer = None;
        self.waitForSpeechDone = waitForSpeechDone;
        # In the following, the term '_settings' would make more
        # sense, but other parts of the code rely on _joints as 
        # the name:
        self._joints = [{'label': 'speak', 
                         'checkboxLabel': 'Stop till speech done',
                         'text': utterance,
                         'waitForSpeechDone': waitForSpeechDone}]
        self.initLocalOperation();
        Pr2SpeakAction.registerSpeechAct(self);
        
    #----------------------------------
    # initLocalOperation 
    #--------------
    
    def initLocalOperation(self):
        
        self.stand_alone = True;
        
        self.ttsProvider = self.toolbox.textToSpeechProvider;
        
        try:
            self.assureTtsEngineAndVoiceAvailable(self.voice, self.ttsEngine);
        except ValueError:
            # Voice or engine unavailable on the machine where execution is running:
            rospy.logwarn("The combination %s/%s is unavailable on this machine. Maybe an unlicensed voice? Or text-to-speech engine not installed? Trying default" % (str(self.voice), str(self.ttsEngine)));
            self.voice = 'voice_kal_diphone';
            self.ttsEngine = 'festival'; 

    #----------------------------------
    # assureTtsEngineAndVoiceAvailable
    #--------------

    def assureTtsEngineAndVoiceAvailable(self, voice, ttsEngine):
        
        # If both voice and engine are not provided, defaults will be used:
        if voice is None and ttsEngine is None:
            return;
        

        engineVoiceDict = self.ttsProvider.availableVoices();        
        if ttsEngine is not None:
            # Ensure that the requested tts engine exists:
            if not ttsEngine in self.ttsProvider.availableTextToSpeechEngines():
                raise ValueError("Engine %s not supported. Supported engines: %s" % (str(ttsEngine), 
                                                                                   str(engineVoiceDict.keys())));
            # Engine available; if voice is given, does this engine support this voice?
            if voice is not None:
                if voice not in engineVoiceDict[ttsEngine]:
                    raise ValueError("Voice %s unavailable in text-to-speech engine %s. Available voices: %s" % (str(voice),
                                                                                                                 str(ttsEngine),
                                                                                                                 str(engineVoiceDict[ttsEngine])))
            return;
        
        # TTS Engine not provided, but voice is provided.
        # Ensure that *some* engine provides the requested voice:
         
        foundVoice = False;
    
        for engine in engineVoiceDict.keys():
            if voice in engineVoiceDict[engine]:
                self.ttsEngine = engine;
                return;
        # No engine provides the voice:
        # Get list of all voices:
        voices = [];
        for voiceList in engineVoiceDict.values():
            voices.extend(voiceList);
        raise ValueError("Voice %s unavailable. Available voices are %s") % (str(voice), str(voices));

    #----------------------------------
    # get_value
    #--------------

    def get_value(self, label):
        try:
            return self._joints[0][label]
        except KeyError:
            raise KeyError('settings with label "%s" not found in speech action.' % label)

    #----------------------------------
    # update_value
    #--------------
    
    def update_value(self, label, value):
        if label == "waitForSpeechDone":
            self.waitForSpeechDone = value
        elif label == "text":
            self.utterance = value
        else:
            return
        #self.waitForSpeechDone, self.utterance = value; 


    # --------------------- Abstract and Override Methods from Action Superclass -----------

    #----------------------------------
    # __repr__
    #--------------
    
    def __repr__(self):
        if len(self.utterance) > 10:
            snippet = self.utterance[:10] + "...";
        else:
            snippet = self.utterance;
        return "<speak['%s' (Eng: %s; Voice: %s)]>" % (str(snippet), str(self.ttsEngine), str(self.voice));

    #----------------------------------
    # to_string
    #--------------

    def to_string(self):
        return "speak(text='%s', voice='%s', wait for speech done: %s)" % (str(self.utterance), str(self.voice), str(self.waitForSpeechDone));

    #----------------------------------
    # dataForTable
    #--------------

#    def dataForTable(self):
#        '''
#        Return string to display in poses table while in editing mode:
#        '''
#        return "%s;'%s'" % ('WAIT' if self.waitForSpeechDone else 'CONTINUE', self.utterance)
    
    #----------------------------------
    # deepcopy 
    #--------------

    def deepcopy(self):
        
        newInst = super(Pr2SpeakAction, self).deepcopy();
        newInst.utterance = self.utterance;
        newInst.voice = self.voice;
        newInst.ttsEngine = self.ttsEngine;
        newInst.waitForSpeechDone = self.waitForSpeechDone;
        #newInst.soundControl = self.soundControl;
        newInst.initLocalOperation();
        
        return newInst;

    #----------------------------------
    # serialize
    #--------------

    def serialize(self, stream):
        super(Pr2SpeakAction, self).serialize(stream);        
    	valueDict = {
    	  'utterance' : self.utterance,
    	  'voice'     : self.voice,
    	  'ttsEngine' : self.ttsEngine,
          'waitForSpeechDone' : self.waitForSpeechDone
    	  }
        stream.serialize_data(valueDict);

    #----------------------------------
    # deserialize
    #--------------
    
    def deserialize(self, stream):
        super(Pr2SpeakAction, self).deserialize(stream);
    	valueDict = stream.deserialize_data();
        self.utterance = valueDict['utterance'];
        self.voice     = valueDict['voice'];
        self.ttsEngine = valueDict['ttsEngine'];
        self.waitForSpeechDone = valueDict['waitForSpeechDone'];
        
        self.initLocalOperation();

    #----------------------------------
    # execute 
    #--------------

    def execute(self):
        # If already playing, the lock is set. Check for
        # this condition without blocking. Return of False
        # means already locked. If that's the case, we stop
        # the current playback, and start the new one. If the
        # lock acquisition succeeded, nothing was playing, and
        # we acquired the lock.
        if not Pr2SpeakAction.multiSpeechBlockLock.acquire(NON_BLOCKING):
            self.stop();
        try:
            
            if self.ttsEngine == 'cepstral':
                # Convert any speech modulation markup to official W3C SSML markup:
                ssmlText = self.convertRawTextToSSML(self.utterance);
            else:
                ssmlText = self.utterance;
            
            # If a previous playback ending removed this Pr2SpeakAction
            # instance from the list of speak actions, add it back in.
            # The instance must be registered so that the stop button
            # will stop playback:
            Pr2SpeakAction.registerSpeechAct(self);
                
            self.ttsProvider.say(ssmlText, 
                                 voiceName=self.voice, 
                                 t2sEngineName=self.ttsEngine, 
                                 blockTillDone=False);
            if not self.waitForSpeechDone:
                # Pretend that this action is done, but remember that it's not
                # by remembering this action instance:
                self.speechExecutionFinished(REMEMBER_SPEAK_INSTANCE);
                return
                
            # Check periodically whether playback is done:
            self._timer = rospy.Timer(rospy.Duration.from_sec(BUSY_CHECK_PERIOD), self.timerFinished, oneshot=True);
        finally:
            try:
                Pr2SpeakAction.multiSpeechBlockLock.release();
            except:
                pass;

    #----------------------------------
    # speechExecutionFinished
    #--------------
   
    def speechExecutionFinished(self, abandonActionInstance):
        if abandonActionInstance:
            Pr2SpeakAction.unregisterSpeechAct(self);
        self._execute_finished();
        
    #----------------------------------
    # convertRawTextToSSML   
    #--------------
   
    def convertRawTextToSSML(self, rawText):
        '''
        Given a string with SpeakEasy speech modulation markup, convert the
        string to W3C SSML marked-up text, and return a new string. Example:
        'This is [P90my] test' --> 'This is <prosody pitch='+90%'>my</prosody> test'.
        Note: Only the Cepstral engine currently handles SSML.
        @param theStr: string to convert
        @type theStr: String
        '''
        try:
            ssmlText = MarkupManagement.convertStringToSSML(rawText);
        except ValueError as e:
            self.dialogService.showErrorMsg(`e`);
        return ssmlText;


    #----------------------------------
    # stop 
    #--------------

    def stop(self, reason=None):
        if reason is None:
            return;
        self.ttsProvider.stop();
        if self._timer is not None:
            self._timer.shutdown();
            self._timer = None;
        self.speechExecutionFinished(FORGET_SPEAK_INSTANCE);

    #----------------------------------
    # timerFinished
    #--------------

    def timerFinished(self, event):
        self._timer = None
        # If speech still going on, just set the timer again, else cause
        # the done signal to be raised:
        if self.ttsProvider.busy():
            self._timer = rospy.Timer(rospy.Duration.from_sec(BUSY_CHECK_PERIOD), self.timerFinished, oneshot=True);
            return;

        self.speechExecutionFinished(FORGET_SPEAK_INSTANCE);

    # -------------------------  Testing --------------------------
    
if __name__ == '__main__':
    
    import unittest
    from sound_dialog import SoundControl
    from python_qt_binding import QtBindingHelper
    from QtGui import QApplication
    import sys
    
    class SpeakActionTest(unittest.TestCase):
        
        def setUp(self):
            self.toolbox = Toolbox.getInstance();
        
        def testToString(self):
            action = Pr2SpeakAction(self.toolbox, "Test convert to string minimal info.");
            print "testToString: " + action.to_string()
            assert(action.to_string() == "speak(text='Test convert to string minimal info.', voice='voice_kal_diphone')")
            
            
            action = Pr2SpeakAction(self.toolbox, "Test convert to string fully qualified.", ttsEngine="cepstral", voice='David');
            assert(action.to_string() == "speak(text='Test convert to string fully qualified.', voice='David')");
  
        def testExecute1(self):
            action = Pr2SpeakAction(self.toolbox, "Test default voice");
            print "testExecute1: " + action.to_string()
            action.execute()
  
        def testExecute2(self):
            action = Pr2SpeakAction(self.toolbox, "Test David voice.", voice='David', ttsEngine='cepstral');
            print "testExecute2: " + action.to_string()
            action.execute()
            
        def testExecute3(self):
            try:
                action = Pr2SpeakAction(self.toolbox, "Test bad voice specification uses default voice.", voice='Foobar');
                print "testExecute3: " + action.to_string()                            
                action.execute()
                raise ValueError("Should have had exception for unavailable voice.")
            except ValueError:
                # Expected exception: David not supported on default engine Festival
                print "Successful exception handling."
                pass
            
    unittest.main();
      
    #print("Test no engine or voice provided.");
#    noVoiceNoEng.execute();
    #print("Done testing no engine or voice provided.");
        
