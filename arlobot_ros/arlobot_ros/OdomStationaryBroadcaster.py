#!/usr/bin/env python
import time
from six.moves import input
import threading
import rclpy
from rclpy.node import Node


def _printOutputFunction(data):
    print(data)


def _EmptyInputHandler():
    print("OdomStationaryBroadcaster called.")


class OdomStationaryBroadcaster(Node):
    """
    Thread to broadcast stationary odometry transform and topic when Propeller board is not initialized
    """

    def __init__(self, broadcaster=_EmptyInputHandler,printOutputFunction=_printOutputFunction):
        #self.r = Node.create_rate(5)  # refresh rate in Hz
        self._StaticOdometrySender = broadcaster
        self._KeepRunning = False
        self._ReceiverThread = None

        self._printOutputFunction = printOutputFunction

    def Start(self):
        self._printOutputFunction("Starting OdomStationaryBroadcaster")
        self._KeepRunning = True
        self._ReceiverThread = threading.Thread(target=self._OdomKicker)
        self._ReceiverThread.setDaemon(True)
        self._ReceiverThread.start()

    def _OdomKicker(self):
        while self._KeepRunning:
            self._StaticOdometrySender()
            time.sleep(0.1)
            #self.r.sleep()  # Sleep long enough to maintain the rate set in __init__

    def Stop(self):
        self._printOutputFunction("Stopping OdomStationaryBroadcaster")
        self._KeepRunning = False


if __name__ == "__main__":
    dataReceiver = OdomStationaryBroadcaster()
    dataReceiver.Start()

    input("Hit <Enter> to end.")
    dataReceiver.Stop()
