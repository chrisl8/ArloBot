#!/usr/bin/env python
import threading
import time
import rospy

def _EmptyInputHandler(line):
    print("OdomStationaryBroadcaster called.")

class OdomStationaryBroadcaster(object):
    '''
    Thread to broadcast stationary odometry transform and topic when Propeller board is not initialized
    '''

    def __init__(self, broadcaster = _EmptyInputHandler):
        self.r = rospy.Rate(5) # refresh rate in Hz
        self._StaticOdometrySender = broadcaster
        self._KeepRunning = False

    def Start(self):
        rospy.loginfo("Starting OdomStationaryBroadcaster")
        self._KeepRunning = True
        self._ReceiverThread = threading.Thread(target=self._OdomKicker)
        self._ReceiverThread.setDaemon(True)
        self._ReceiverThread.start()

    def _OdomKicker(self):
        while self._KeepRunning:
            self._StaticOdometrySender()
            self.r.sleep() # Sleep long enough to maintain the rate set in __init__
    
    def Stop(self):
        rospy.loginfo("Stopping OdomStationaryBroadcaster")
        self._KeepRunning = False

    if __name__ == '__main__':
        dataReceiver = OdomStationaryBroadcaster()
        dataReceiver.Start()

        raw_input("Hit <Enter> to end.")
        dataReceiver.Stop()
