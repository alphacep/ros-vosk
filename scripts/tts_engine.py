#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import pyttsx3

import rospy
from std_msgs.msg import String
from std_msgs.msg import Bool

class tts_engine():

    def __init__(self):
        rospy.Subscriber("tts/phrase", String, self.callback)
        self.pubStatus = rospy.Publisher('tts/status', Bool, queue_size=0)

    def callback(self, msg):
        phrase = msg.data
        self.say(phrase)
    
    def publish_status(self, isSpeaking):
        # Make the status true if it is speaking and false if it is not
        if isSpeaking == True:
            rospy.sleep(0.1)
            self.pubStatus.publish(True)
            rospy.logdebug("Started Speaking!")
        else:
            rospy.sleep(0.1)
            self.pubStatus.publish(False)
            rospy.logdebug("Finished Speaking..")
    
    def say(self, phrase):

        rospy.loginfo("The robot says: " + phrase)
        self.engine = pyttsx3.init()
        self.engine.connect('started-utterance', self.tts_onStart)
        self.engine.connect('finished-utterance', self.tts_onEnd)
        self.engine.say(phrase,"tts_engine")
        self.engine.runAndWait()

    def tts_onStart(self, name):
        rospy.logdebug('starting speaking ', name)
        self.publish_status(True)

    def tts_onWord(self, name, location, length):
        rospy.logdebug('word', name, location, length)

    def tts_onEnd(self, name, completed):
        rospy.logdebug('finishing speaking', name, completed)
        self.publish_status(False)
        self.engine.endLoop()
   
if __name__ == '__main__':
    try:
        rospy.init_node('tts_engine', anonymous=True)
        tts = tts_engine()
        rospy.spin() 
    except KeyboardInterrupt:
        rospy.loginfo("Stopping tts engine...")
        rospy.sleep(1)
        print("node terminated")