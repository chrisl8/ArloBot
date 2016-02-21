#!/usr/bin/env python
# Software License Agreement (BSD License)
#
# Author: Chris L8 https://github.com/chrisl8
# URL: https://github.com/chrisl8/ArloBot

import rospy
import tf
import sys
import time

from std_msgs.msg import String
from std_msgs.msg import Bool
from arlobot_msgs.msg import usbRelayStatus
from arlobot_msgs.srv import *

#For USB relay board
from pylibftdi import BitBangDevice
from pylibftdi import Driver # Required to get serial number of unknown USB Relay device.

# From:
# ----------------------------------------------------------------------------
#
#   DRCONTROL.PY
#
#   Copyright (C) 2012 Sebastian Sjoholm, sebastian.sjoholm@gmail.com
#
#   This program is free software: you can redistribute it and/or modify
#   it under the terms of the GNU General Public License as published by
#   the Free Software Foundation, either version 3 of the License, or
#   (at your option) any later version.
#
#   This program is distributed in the hope that it will be useful,
#   but WITHOUT ANY WARRANTY; without even the implied warranty of
#   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#   GNU General Public License for more details.
#
#   You should have received a copy of the GNU General Public License
#   along with this program.  If not, see <http://www.gnu.org/licenses/>.
#
#   Version history can be found at
#   http://code.google.com/p/drcontrol/wiki/VersionHistory
#
#   $Rev$
#   $Date$
#
# ----------------------------------------------------------------------------
# For SainSmart 8 port USB model http://www.sainsmart.com/sainsmart-4-channel-12-v-usb-relay-board-module-controller-for-automation-robotics-1.html
class relay_data(dict):

    address = {
            "1":"1",
            "2":"2",
            "3":"4",
            "4":"8",
            "5":"10",
            "6":"20",
            "7":"40",
            "8":"80",
            "all":"FF"
            }

    def __getitem__(self, key): return self[key]
    def keys(self): return self.keys()

# ----------------------------------------------------------------------------
# testBit() returns a nonzero result, 2**offset, if the bit at 'offset' is one.
# http://wiki.python.org/moin/BitManipulation
# ----------------------------------------------------------------------------

def testBit(int_type, offset):
    mask = 1 << offset
    return(int_type & mask)

# ----------------------------------------------------------------------------
# LIST_DEVICES()
#
# Routine modified from the original pylibftdi example by Ben Bass
# ----------------------------------------------------------------------------

def list_devices():
    print "Vendor\t\tProduct\t\t\tSerial"
    dev_list = []
    for device in Driver().list_devices():
        device = map(lambda x: x.decode('latin1'), device)
        vendor, product, serial = device
        print "%s\t\t%s\t\t%s" % (vendor, product, serial)

def return_device_serial_number():
    #print "Vendor\t\tProduct\t\t\tSerial"
    dev_list = []
    for device in Driver().list_devices():
        device = map(lambda x: x.decode('latin1'), device)
        vendor, product, serial = device
        #print "%s\t\t%s\t\t%s" % (vendor, product, serial)
    return serial

# For SainSmart 8 port USB model http://www.sainsmart.com/sainsmart-4-channel-12-v-usb-relay-board-module-controller-for-automation-robotics-1.html
def get_relay_state( data, relay ):
    if relay == "1":
        return testBit(data, 0)
    if relay == "2":
        return testBit(data, 1)
    if relay == "3":
        return testBit(data, 2)
    if relay == "4":
        return testBit(data, 3)
    if relay == "5":
        return testBit(data, 4)
    if relay == "6":
        return testBit(data, 5)
    if relay == "7":
        return testBit(data, 6)
    if relay == "8":
        return testBit(data, 7)

class UsbRelay(object):
    '''
    Helper class for communicating with a Propeller board over serial port
    '''

    def __init__(self):
        rospy.init_node('arlobot_usbrelay')
        # http://wiki.ros.org/rospy_tutorials/Tutorials/WritingPublisherSubscriber
        self.r = rospy.Rate(0.25) # 1hz refresh rate
        #self.r = rospy.Rate(1) # 1hz refresh rate

        # Prevent overlapping calls to the serial port
        self._Busy = False

        # Wait for the arlobot_bringup launch file to initiate the usbRelayInstalled parameter before starting:
        while not rospy.has_param('/arlobot/usbRelayInstalled') and not rospy.is_shutdown():
            rospy.loginfo("arlobot_bringup not started yet, waiting . . .")
            rospy.sleep(1)

        self.relayExists = rospy.get_param("/arlobot/usbRelayInstalled", False)
        if self.relayExists:
            #list_devices()
            #NOTE: This will return the LAST device, so if you have multiple USB Relay boards this will have to be modified
            self.relaySerialNumber = return_device_serial_number()

        else:
            rospy.loginfo("No USB Relay board installed.")

        # Create a service that can be called to toggle any relay by name:
        # http://wiki.ros.org/ROS/Tutorials/CreatingMsgAndSrv
        # http://wiki.ros.org/ROS/Tutorials/WritingServiceClient%28python%29
        relayToggle = rospy.Service('~toggle_relay', ToggleRelay, self._ToggleRelayByName)
        relayFind = rospy.Service('~find_relay', FindRelay, self._FindRelayByName)

        # Publishers
        self._usbRelayStatusPublisher = rospy.Publisher('~usbRelayStatus', usbRelayStatus, queue_size=1) # for publishing status of USB Relays

    def _FindRelayByName(self, req):
        # This function will return the relay number for a given name based on the usbrelay.yaml loaded parameters
        # In theory any node can do this, but it helps to make this service available, since the topic we publish requires
        # You to know the number of the relay to figure out which array entry is the one you want.
        boardExists = False
        foundRelay = False
        relayNumber = 0
        if self.relayExists and not rospy.is_shutdown():
            if self.relayExists: # Do not do this if no relay exists.
                # Toggle Relay
                boardExists = True
                for i in range(1,9): # Walk the relays to find the one with the right name.
                    relayEnabled = rospy.get_param("~relay" + str(i) + "enabled", False)
                    if relayEnabled: # Do not touch (poll or anything) Relays that are not listed as enabled.
                        relayLabel = rospy.get_param("~relay" + str(i) + "label", "No Label")
                        if relayLabel == req.relay:
                            foundRelay = True
                            relayNumber = i
        return(boardExists, foundRelay, relayNumber)

    def _ToggleRelayByName(self, req):
        boardExists = False
        foundRelay = False
        toggleSuccess = False
        if self.relayExists: # Do not do this if no relay exists.
            # Toggle Relay
            boardExists = True
            for i in range(1,9): # Walk the relays to find the one with the right name.
                relayEnabled = rospy.get_param("~relay" + str(i) + "enabled", False)
                if relayEnabled: # Do not touch (poll or anything) Relays that are not listed as enabled.
                    relayLabel = rospy.get_param("~relay" + str(i) + "label", "No Label")
                    if relayLabel == req.relay:
                        foundRelay = True
                        while self._Busy: # Prevent simultaneous polling of serial port by multiple processes within this app due to ROS threading.
                            rospy.loginfo("BitBangDevice Busy . . .")
                            rospy.sleep(0.4)
                        self._Busy = True
                        rospy.loginfo("Changing relay " + str(self.relaySerialNumber) + " to " + str(req.state))
                        if req.state:
                            BitBangDevice(self.relaySerialNumber).port |= int(relay_data.address[str(i)], 16)
                            #BitBangDevice(relaySerialNumber).port |= int(relay.address[leftMotorRelay], 16)
                        elif not req.state:
                            BitBangDevice(self.relaySerialNumber).port &= ~int(relay_data.address[str(i)], 16)
                        checkState = get_relay_state( BitBangDevice(self.relaySerialNumber).port, str(i) )
                        self._Busy = False
                        if checkState == 0:
                            newState = False
                        else:
                            newState = True
                        if newState == req.state:
                            toggleSuccess = True
        return(boardExists, foundRelay, toggleSuccess)

    def Run(self):
        # Get and broadcast status of all USB Relays.
        while not rospy.is_shutdown():
            relaystatus = usbRelayStatus()
            relaystatus.relayOn = [False] * 8 # Fill array for use.
            if self.relayExists: # Only poll if the relay exists.
                relaystatus.relayPresent = True
                # Gather USB Relay status for each relay and publish
                for i in range(1,9):
                    if not rospy.is_shutdown(): # ROS tends to get shut down while this is looping and throw a lot of errors
                        relayEnabled = rospy.get_param("~relay" + str(i) + "enabled", False)
                        if relayEnabled: # Do not touch (poll or anything) Relays that are not listed as enabled.
                            relayLabel = rospy.get_param("~relay" + str(i) + "label", "No Label")
                            while self._Busy: # Prevent simultaneous polling of serial port by multiple processes within this app due to ROS threading.
                                rospy.loginfo("BitBangDevice Busy . . .")
                                rospy.sleep(0.2)
                            self._Busy = True
                            state = get_relay_state( BitBangDevice(self.relaySerialNumber).port, str(i) )
                            self._Busy = False
                            if state == 0:
                                #print "Relay " + str(i) + " state:\tOFF (" + str(state) + ")"
                                #print str(i) + " - " + relayLabel + ": OFF"
                                relaystatus.relayOn[i-1] = False
                            else:
                                #print "Relay " + str(i) + " state:\tON (" + str(state) + ")"
                                #print str(i) + " - " + relayLabel + ": ON"
                                relaystatus.relayOn[i-1] = True
            else: # If the relay does not exist just broadcast "False" to everything.
                relaystatus.relayPresent = False
            self._usbRelayStatusPublisher.publish(relaystatus) # Publish USB Relay status
            self.r.sleep() # Sleep long enough to maintain the rate set in __init__

    def Stop(self):
        rospy.loginfo("Shutting off all relays . . .")
        # At this point ROS is shutting down, so any attempts to check parameters or log may crash.
        self._Busy = True # We are shutting down, so make everyone else stall and plow ahead:
        for i in range(1,9): # Walk the relays
            state = get_relay_state( BitBangDevice(self.relaySerialNumber).port, str(i) )
            while not state == 0: # Loop if it doesn't shut off.
                BitBangDevice(self.relaySerialNumber).port &= ~int(relay_data.address[str(i)], 16)
                state = get_relay_state( BitBangDevice(self.relaySerialNumber).port, str(i) )

if __name__ == '__main__':
    node = UsbRelay()
    rospy.on_shutdown(node.Stop)
    try:
        node.Run()
    except rospy.ROSInterruptException:
        node.Stop()
