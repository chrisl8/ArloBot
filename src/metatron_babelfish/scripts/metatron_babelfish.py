#!/usr/bin/env python
# pylint: disable=line-too-long
#TODO: This requires the Ubuntu espeak package, but I'm not sure how to set up that requirement.

import rospy
import subprocess
import random
from std_msgs.msg import Bool
# You CANNOT declare the services in the same package as the node that implements it!
# So this cannot work:
#from metatron_babelfish.srv import *
# If you do you will get "ImportError: No module named srv"
from metatron_services.srv import SpeakText, ListenText
# For SMS sending
from twilio.rest import TwilioRestClient
import os.path
import time

class MetatronBabel(object):

    def __init__(self):
        rospy.init_node('metatron_babelfish')
        # http://wiki.ros.org/rospy_tutorials/Tutorials/WritingPublisherSubscriber
        self.r = rospy.Rate(1) # 1hz refresh rate

        self.isSpeaking = False
        self.speechEngine = 'espeak' # Use espeak by default because it comes with Ubuntu
        onlineSpeechEngine = '/home/chrisl8/speech/attText2Speech.sh'
        if os.path.isfile(onlineSpeechEngine):
            self.speechEngine = onlineSpeechEngine
        self.soundFileLocation = rospy.get_param("~dictionary/soundFileLocation", "/")
        #print self.speechEngine

        # Set laptop volume to 100% so everyone can hear us
        process = subprocess.Popen(['amixer', '-q', 'set', 'Master', '100%'], close_fds=True)
        process.wait() # Wait for it to finish, this should be instantaneous.
        # TODO This could get reset by something else, it might be smart to do this at every output,
        # but that would get old, and prevent us from shutting him up!

        #Twilio setup
        self.use_twilio = rospy.get_param("/metatron_id/use_twilio", False)
        self.my_phone_number = rospy.get_param("/metatron_id/my_phone_number", "")
        self.twilio_account_sid = rospy.get_param("/metatron_id/twilio_account_sid", "")
        self.twilio_auth_token = rospy.get_param("/metatron_id/twilio_auth_token", "")
        self.twilio_number = rospy.get_param("/metatron_id/twilio_number", "")

        # In case we cannot find somethig in the dictionary, have something to say.
        self.missingDictionaryItem = ['I really have no idea what to say.']

        # Track changes in AC power status
        self.acPower = -1
        self.acPower_count = 0 # Track duration of current trend

        # Passive Infrared Sensor (PIR) monitor
        #The Activity Board code only polls this while the robot is idle.
        #This is kind of a way to check for activity in the room and
        #interact when the robot is not operating.
        self.PIR_last_result = False # Keep track of last result
        self.PIR_last_count = 0 # Keep track of duration of current trend
        self.PIR_last_hello = rospy.get_time()
        self.lastStatement = rospy.get_time()
        rospy.Subscriber('/arlobot/pirState', Bool, self._PIRresponder) # Subscribe to topic published by arlobot_bringup

        # Create a service that can be called to speak any text
        # http://wiki.ros.org/ROS/Tutorials/CreatingMsgAndSrv
        # http://wiki.ros.org/ROS/Tutorials/WritingServiceClient%28python%29
        speaker = rospy.Service('metatron_speaker', SpeakText, self._handle_speak_request)

        # Create a service that can be called to accept any text and deal with it
        listener = rospy.Service('metatron_listener', ListenText, self._handle_listen_request)

        # Introduction
        text = rospy.get_param("~dictionary/introduction", self.missingDictionaryItem)
        text = random.choice(text)
        self._say(text)

    def _PIRresponder(self, topicInput): # Pattern for using a variable to track changes in a topic
        # For debugging the motion sensing parameters:
        #print "PIR: " +str(topicInput.data) + " Previous: " + str(self.PIR_last_result) + " " + str(self.PIR_last_count) + " AC: " + str(self.acPower) + " " + str(self.acPower_count) + " Last Hello: " + str(rospy.get_time() - self.PIR_last_hello) +" Last Statement: " + str(rospy.get_time() - self.lastStatement)
        if self.PIR_last_result != topicInput.data: # What to do if status changes
            if topicInput.data: # If PIR is seeing something
                # Only "query" if it has been false/still for a while, otherwise we are kind of repeating ourselves
                if self.PIR_last_count > 5:
                    if rospy.get_time() - self.PIR_last_hello > 600: # Initial greeting squelch in seconds
                        text = rospy.get_param("~dictionary/PIRsensor1st", self.missingDictionaryItem)
                        text = random.choice(text)
                        print text
                        self._say(text)
                        self.PIR_last_hello = rospy.get_time()
            else: # If PIR is no longer seeing something
                if self.PIR_last_count > 15: # Only say "bye" if the activity was significant
                    text = rospy.get_param("~dictionary/PIRsensorOff", self.missingDictionaryItem)
                    text = random.choice(text)
                    print text
                    self._say(text)
            self.PIR_last_result = topicInput.data # New "last_result"
            self.PIR_last_count = 0 # Restart count since this is a new status
        else: # Same result as last time
            if topicInput.data:
                if self.PIR_last_count == 15: # Second statement, if someone has been around for a while
                    text = rospy.get_param("~dictionary/PIRsensor2nd", self.missingDictionaryItem)
                    text = random.choice(text)
                    #print text
                    #self._say(text) # Second text turned off for now.
            self.PIR_last_count += 1

    def Stop(self):
        rospy.loginfo("Metatron BabelFish is shutting down.")

    def Chatter(self):
        while not rospy.is_shutdown():
            # Check status of AC Power
            state = rospy.get_param("/arlobot_safety/ACpower", True)
            if self.acPower == -1:
                self.acPower = state
                self.acPower_count = 0
                rospy.loginfo("acPower initial set to " + str(self.acPower))
            elif self.acPower != state:
                if state:
                    text = rospy.get_param("~dictionary/pluggedIn", self.missingDictionaryItem)
                    text = random.choice(text)
                    self._say(text)
                    self.acPower = True
                else: # I think with a Bool this should work, as there is no 3rd option, no?
                    text = rospy.get_param("~dictionary/unPlugged", self.missingDictionaryItem)
                    text = random.choice(text)
                    self._say(text)
                    self.acPower = False
                self.acPower_count = 0 # Reset trend
            self.acPower_count += 1

            #Random chatter
            if self.PIR_last_result:
                delayTime = 60 * 5 # Talk more often if people are around
            else:
                delayTime = 60 * 60 # But still talk if no one is around
            if rospy.get_time() - self.lastStatement > delayTime:
                text = rospy.get_param("~dictionary/randomChatter", self.missingDictionaryItem)
                text = random.choice(text)
                self._say(text)
            self.r.sleep() # Sleep long enough to maintain the rate set in __init__

    def _handle_speak_request(self, text):
        # You can run this to test this service from the command line:
        #rosservice call /metatron_speaker "Hello world"
        # and it should return:
        #speach_result: True
        self._say(text.text_to_speak)
        return True

    def _handle_listen_request(self, text):
        print text.input_text
        print text.sender[1:]
        return True

    def _say(self, text):
        while self.isSpeaking:
            time.sleep(1)
        self.isSpeaking = True
        is_wav = False
        if len(text.split('.')) > 1:
            if text.split('.')[1] == 'wav':
                is_wav = True
        if is_wav:
            process = subprocess.Popen(['/usr/bin/aplay', '-q', self.soundFileLocation + "/" + text], close_fds=True)
        else:
            process = subprocess.Popen([self.speechEngine, text], close_fds=True)
        process.wait() # Wait for it to finish, this should be instantaneous.

        if not is_wav & self.use_twilio:
            client = TwilioRestClient(self.twilio_account_sid, self.twilio_auth_token)
            client.messages.create(
                to=self.my_phone_number,
                from_=self.twilio_number,
                body=text,
            )
        self.isSpeaking = False
        self.lastStatement = rospy.get_time()

if __name__ == '__main__':
    node = MetatronBabel()
    rospy.on_shutdown(node.Stop)
    try:
        node.Chatter()
    except rospy.ROSInterruptException:
        node.Stop()
