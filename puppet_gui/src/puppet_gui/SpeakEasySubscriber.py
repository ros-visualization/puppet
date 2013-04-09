#!/usr/bin/env python

import roslib
roslib.load_manifest('puppet_gui')

import rospy
from speakeasy.msg import SpeakEasyButtonProgram
from Signal import Signal
from toolbox_gui import CommChannel

class SpeakEasySubscriber(object):
         
    def __init__(self):
        rospy.loginfo("Starting SpeakEasy/Puppet link")
        self.commChannel = CommChannel()
        rospy.init_node('speakEasyPuppetLink', anonymous=True)
        rospy.Subscriber("SpeakEasyButtonProgram", SpeakEasyButtonProgram, self.speakEasyProgramArrived)
        rospy.spin()
    
    def speakEasyProgramArrived(self, programMsg):
        whatToSay  = programMsg.text
        voiceToUse = programMsg.voiceName
        ttsEngine  = programMsg.engineName
        print("What: %s. Voice: %s. Engine: %s" % (whatToSay,voiceToUse,ttsEngine))
        self.commChannel.insertRobotSaysSignal.emit(whatToSay, voiceToUse, ttsEngine)
    
if __name__ == '__main__':
    SpeakEasySubscriber()    