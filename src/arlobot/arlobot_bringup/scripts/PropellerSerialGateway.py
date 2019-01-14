#!/usr/bin/env python

# ======================================
# Propeller Serial Gateway
# ======================================
"""
This code opens up and maintains a continuous serial connection
with the Propeller board.

It will open a serial connection when requested,
send data when requested,
listen for incoming data and call a given function,
reconnect to the device if the connection is lost,
and shut down the connection when asked to.

This script is typically not called directly, but rather
is called by PropellerSerialInterface.py,
although you CAN call this directly just for testing,
in which case it will echo whatever comes in to the screen.

Inspired by similar code created by Dr. Rainer Hessmer
on November 20, 2010
"""

import threading
import serial
import time
import rospy


def _OnLineReceived(line):
    """
    This is only used when this script is run directly,
    which is only done for testing purposes.
    It just echos whatever comes in to the screen.
    """
    print line


def _printOutputFunction(data):
    print data


# noinspection PyBroadException
class PropellerSerialGateway(object):
    """
    Helper class for receiving lines from a serial port
    """

    def __init__(self, port="/dev/ttyUSB0", baudrate=115200, lineHandler=_OnLineReceived,
                 printOutputFunction=_printOutputFunction, logPrefix="SERIAL PORT "):
        """
        Initializes the receiver class.
        port: The serial port to listen to.
        receivedLineHandler: The function to call when a line was received.
        """
        self._Port = port
        self._Baudrate = baudrate
        self.ReceivedLineHandler = lineHandler
        self._KeepRunning = False
        self._ConnectedToSerialDevice = False
        self._LogPrefix = logPrefix
        self._printOutputFunction = printOutputFunction

        # Define the Watchdog thread
        self._ReceiverThread = threading.Thread(target=self._Watchdog)

    def _Watchdog(self):
        # After Start() is run, this program attempts to keep the port connected as long as it is running.
        while self._KeepRunning:
            if self._ConnectedToSerialDevice:
                # _Listen will deal with disconnecting if there is an error.
                self._Listen()
            else:
                time.sleep(1)
                self._Connect()

    def Start(self):
        # Turn on the "Keep running" semaphore
        self._KeepRunning = True
        # Start the watchdog thread
        self._ReceiverThread.setDaemon(True)
        self._ReceiverThread.start()

    def _Connect(self):
        try:
            # sys.tracebacklimit = 0
            self._Serial = serial.Serial(
                port=self._Port, baudrate=self._Baudrate, timeout=0, write_timeout=0)
            self._printOutputFunction(self._LogPrefix + str(self._Port) +
                                      " connected at " + str(self._Baudrate))
            self._ConnectedToSerialDevice = True
        except:
            self._printOutputFunction(
                self._LogPrefix + "Unable to connect to " + str(self._Port) + ", will retry...")
            rospy.loginfo("Unable to connect to " +
                          str(self._Port) + ", will retry...")

    def Stop(self):
        # Stop the Watchdog
        self._KeepRunning = False
        # Disconnect
        self._Disconnect()

    def _Disconnect(self):
        self._printOutputFunction(self._LogPrefix + "DISconnecting")
        rospy.loginfo(self._LogPrefix + "DISconnecting")
        try:
            self._Serial.close()
            self._ConnectedToSerialDevice = False
        except:
            self._printOutputFunction(self._LogPrefix + "Stop Error")
            rospy.loginfo(self._LogPrefix + "Stop Error")

    def _Listen(self):
        data = ''  # Initialize.
        try:
            data = self._Serial.read(1)
            # If there is no data on the line it will return ''
        except:
            self._printOutputFunction(self._LogPrefix + "Listen Error")
            rospy.loginfo(self._LogPrefix + "Listen Error")
            self._Disconnect()
        if data == '':
            # Pause to avoid hogging the CPU
            time.sleep(0.1)
            # NOTE: You can easily see in top/htop how the CPU usage goes up
            #       as your sleep duration is decreased.
            #       On this laptop:
            #       0.0001 = 13.5% CPU Usage
            #       0.001  = 10.0% CPU usage
            #       0.01   =  1.3% CPU usage (htop itself competes for the top slot)
            #       0.1    =  0.7% CPU usage (hard to find under basic services)
            # NOTE: Just make sure this isn't slowing down input from robot.
        else:
            self.ReceivedLineHandler(data)

    def PropellerReady(self):
        if self._ConnectedToSerialDevice:
            # return self._Serial.out_waiting < 1 and self._Serial.in_waiting < 1
            try:
                return self._Serial.out_waiting == 0
            except:
                self._Disconnect()
                return False
        else:
            return False

    def Write(self, data):
        if self._ConnectedToSerialDevice:
            try:
                self._Serial.write(data)
            except:
                rospy.loginfo(self._LogPrefix + "Write Error")
                self._Disconnect()
        else:
            self._printOutputFunction(self._LogPrefix + "Not Open Yet")
            rospy.loginfo(self._LogPrefix + "Not Open Yet")


if __name__ == '__main__':
    dataReceiver = PropellerSerialGateway()
    dataReceiver.Start()

    raw_input("Hit <Enter> to end.\n")
    dataReceiver.Stop()
