#!/usr/bin/env python

import roslib
roslib.load_manifest('puppet_gui')

import rospy
from speakeasy.msg import speakeasy_button_program
from Signal import Signal
from toolbox_gui import CommChannel

class SpeakEasySubscriber(object):
         
    def __init__(self, signalToRaise):
        rospy.loginfo("Starting SpeakEasy/Puppet link")
        self.signalToRaise = signalToRaise
        #****self.commChannel = CommChannel()
        
        # For stand-alone testing:
        #rospy.init_node('speakEasyPuppetLink', anonymous=True)
        
        rospy.Subscriber("SpeakEasyButtonProgram", speakeasy_button_program, self.speakEasyProgramArrived)
        
        # For stand-alone testing:
        #rospy.spin()
    
    def speakEasyProgramArrived(self, programMsg):
        whatToSay  = programMsg.text
        voiceToUse = programMsg.voiceName
        ttsEngine  = programMsg.engineName
        print("What: %s. Voice: %s. Engine: %s" % (whatToSay,voiceToUse,ttsEngine))
        self.signalToRaise.emit(whatToSay, voiceToUse, ttsEngine)
