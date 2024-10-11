#!/usr/bin/env python

# ======================================
# Propeller Serial Test
# ======================================
"""
This script is meant to test all functions of
both the Propeller Board Serial Interface
and the Arlobot itself by sending it
direct commands over the serial interface
and monitoring output directly from the serial
interface.
"""

from __future__ import print_function

import six
import yaml
import os
import sys
import struct
import time
import curses
import threading
from math import sqrt, pow, radians, pi
from PropellerSerialInterface import PropellerSerialInterface
from PropellerSerialDataPacketTypes import PropellerSerialDataPacketTypes
from PropellerSerialTestCursesInterface import Screen
from checkPropellerCodeVersionNumber import checkPropellerCodeVersionNumber


class Linear(object):
    # Pieces needed for Twist Object below
    x = 0.0
    y = 0.0
    z = 0.0

    def __init__(self, x, y, z):
        self.x = x
        self.y = y
        self.z = z


class Angular(object):
    # Pieces needed for Twist Object below
    x = 0.0
    y = 0.0
    z = 0.0

    def __init__(self, x, y, z):
        self.x = x
        self.y = y
        self.z = z


class Twist(object):
    # Fake Twist Object to simulate use without ROS
    linear = Linear
    angular = Angular

    def __init__(self, linear, angular):
        self.linear = linear
        self.angular = angular


class PropellerSerialTest(object):
    def __init__(self, usbPort="/dev/ttyUSB0"):
        self._usbPort = usbPort

        configFilename = os.getenv("HOME") + "/.arlobot/arlobot.yaml"

        with open(configFilename, "r") as stream:
            try:
                yamlOutput = yaml.safe_load(stream)
            except yaml.YAMLError as exc:
                print(exc)
        self._baudRate = yamlOutput["baudRate"]
        self._driveGeometry = yamlOutput["driveGeometry"]

        self.serialInterface = PropellerSerialInterface(
            self._propellerReadyResponseFunction,
            self._propellerOdomDataHandler,
            self._propellerConfigDataHandler,
            self.TestDataResponseFunction,
            self.sendOutputToScreen,
            self._usbPort,
            self._baudRate,
        )
        self.dataTypes = PropellerSerialDataPacketTypes()

        self.testUnsignedLong = 54732
        self.testIntOne = 958
        self.testIntTwo = -654
        self.testIntThree = 255  # Matches end marker, for testing
        self.testByte = 20
        self.testCharacter = "x".encode("ascii")
        self.testFloat = 20589.48

        self.testData = self.dataTypes.TestDataPacket(
            self.testUnsignedLong,
            self.testIntOne,
            self.testIntTwo,
            self.testIntThree,
            self.testByte,
            self.testCharacter,
            self.testFloat,
        )
        self.errorCount = 0
        self.serialWriteDelay = "?"
        self.goodPacketDelay = "?"
        self.testPacketsReceived = 0
        self.testPacketsSent = 0
        # TODO: Make this switchable from the curses interface.
        self._printIncomingTestPackets = True
        self._printOutgoingTestPackets = True

        self._lastInputCommand = "none"
        self._closingMessageToDisplay = ""
        self.screen = Screen([], self.inputFromScreen)

        self._stallForInit = False

        # Store the data we want to set, or have set from the start
        self._settings = {
            "X": 0.0,
            "Y": 0.0,
            "Heading": 0.0,
            "trackWidth": self._driveGeometry["trackWidth"],
            "distancePerCount": self._driveGeometry["distancePerCount"],
            "wheelSymmetryError": self._driveGeometry["wheelSymmetryError"],
            "ignoreProximity": 0,
            "ignoreCliffSensors": 0,
            "ignoreIRSensors": 0,
            "ignoreFloorSensors": 0,
            "pluggedIn": 1,
            "ledStatus": [],
        }

        # Store the incoming data from the Propeller board
        # Config is stuff we set that either only changes when we tell it to form this end,
        # or at best changes very rarely
        self._config = {
            "trackWidth": 0,
            "distancePerCount": 0,
            "wheelSymmetryError": 0,
            # Set to opposites to  of self._settings
            # to ensure _settingsUpdateRequired stays true
            # until we get the data from the board.
            "ignoreProximity": 1 - self._settings["ignoreProximity"],
            "ignoreCliffSensors": 1 - self._settings["ignoreCliffSensors"],
            "ignoreIRSensors": 1 - self._settings["ignoreIRSensors"],
            "ignoreFloorSensors": 1 - self._settings["ignoreFloorSensors"],
            "pluggedIn": 1 - self._settings["pluggedIn"],
        }
        # Telemetry is stuff that updates rapidly from sensors on the robot
        self._telemetry = {
            "X": 0.0,
            "Y": 0.0,
            "Heading": 0.0,
            "gyroHeading": 0.0,
            "V": 0.0,
            "Omega": 0.0,
            "leftMotorPower": 0.0,
            "rightMotorPower": 0.0,
            "abd_speedLimit": 0,
            "abdR_speedLimit": 0,
            "minDistanceSensor": 0,
            "safeToProceed": 0,
            "safeToRecede": 0,
            "Escaping": 0,
            "cliff": 0,
            "floorO": 0,
            "ledStatus": [],
        }

        self._settingsUpdateRequired = True
        self._ledUpdateRequired = False

        self._previous_position = {"X": 0.0, "Y": 0.0}
        self._previous_heading = 0
        self._navigating_distance = False
        self._navigating_rotation = False
        self._goal_distance = 0.0
        self._goal_rotation = 0.0
        self._previous_difference = 0.0
        self._current_maneuver_command = "stop"
        self._maneuver_rotation_reversed = False

        # Starting minimum move speed
        # 20 ticks per second (TPS) is about 0.07 meters per second,
        # and is about the slowest the Arlo platform can work reliably at.
        self.last_move_speed = 0.08
        self.last_move_turn = 0.3
        self._saved_move_turn = 0.0

        self.run()

    def inputFromScreen(self, data):
        self._lastInputCommand = data

    def sendOutputToScreen(self, data):
        updateStatus = False
        data = str(data)
        if "error" in data.lower():
            self.errorCount += 1
            updateStatus = True
        if "Serial Write Delay:" in data:
            self.serialWriteDelay = data.split(": ")[1]
            updateStatus = True
        if "Good Packet Delay:" in data:
            self.goodPacketDelay = data.split(": ")[1]
            updateStatus = True
        if updateStatus:
            self.screen.inputStatusItems(
                self.errorCount,
                self.serialWriteDelay,
                self.goodPacketDelay,
                self.testPacketsSent,
                self.testPacketsReceived,
            )
        self.screen.addLine(data)

    def _propellerReadyResponseFunction(self, data):
        # When the Propeller Board first boots it will send a 'ready' message
        # until it gets init data.

        # This is only intended for testing. Your calling script should provide
        # an initResponseFunction to provide the proper data to initialize the robot.
        # Initialize so it stops sending the 'r' Ready signal, and stops the pre-init pause

        self._stallForInit = True
        # NOTE: Other fast writing functions should have at least a 1 second sleep
        # if they see self._stallForInit, otherwise they can saturate the interface
        # so badly that an init can never get through if needed after
        # an unexpected board reset or disconnect
        self.screen.addLine("Ready Packet Received:" + str(data))

        # The Ready data includes a version number.
        # Now we should compare it to the version number in the code
        # on this computer to ensure that any updates made here
        # were sent to the Propeller board before this code was run.
        # If not, fail and display an error.
        propellerVersionNumber = checkPropellerCodeVersionNumber()
        self.screen.addLine(
            "Version matches: "
            + str(self.serialInterface.propellerCodeVersion == propellerVersionNumber)
        )
        if not self.serialInterface.propellerCodeVersion == propellerVersionNumber:
            self._closingMessageToDisplay = "ERROR: Propeller Code does not match ROS Code!!!\nPlease use SimpleIDE to install the latest Propeller code to your Propeller Board: https://ekpyroticfrood.net/?p=165"
            self._lastInputCommand = "quit"

        initData = self.dataTypes.InitDataPacket(
            self._settings["X"], self._settings["Y"], self._settings["Heading"]
        )
        self.serialInterface.SendToPropellerOverSerial("init", initData, True)
        time.sleep(0.1)
        self._stallForInit = False

        # NOTE You MUST also send Settings data after an INIT!
        # You CAN set the variable flagging to send settings, but what if
        # it sends ONE and the Propeller misses it?
        # The most sure fire way is to invalidate the config data we have,
        # forcing it to send until it is reset.
        self._config["trackWidth"] = 0
        self._config["distancePerCount"] = 0
        self._config["wheelSymmetryError"] = 0
        # Set to opposites of self._settings
        # to ensure _settingsUpdateRequired stays true
        # until we get the data from the board.
        self._config["ignoreProximity"] = 1 - self._settings["ignoreProximity"]
        self._config["ignoreCliffSensors"] = 1 - self._settings["ignoreCliffSensors"]
        self._config["ignoreIRSensors"] = 1 - self._settings["ignoreIRSensors"]
        self._config["ignoreFloorSensors"] = 1 - self._settings["ignoreFloorSensors"]
        self._config["pluggedIn"] = 1 - self._settings["pluggedIn"]

    def TestSendSettingsFunction(self):
        """
        The Settings function is similar to the Init function, except that it excludes resetting
        the X, Y, and Heading, as those should be tracked as part of the odometry now.
        This is mostly telling the Propeller Board that we are UnPlugged,
        and turning sensor ignore options on/off,
        although it also lets us modify the trackWidth and distancePerCount on the fly, although
        that should be a rare thing to do.

        NOTE: The assumption here is that you UPDATE the internal variable and then just call this
        to update the Propeller board with the current state.

        You could even use a loop somewhere that checks the incoming data from odom,
        compares it to the init state we want,
        and keeps running this function until they match,
        in case of serial errors.
        """
        settingsData = self.dataTypes.SettingsDataPacket(
            self._settings["trackWidth"],
            self._settings["distancePerCount"],
            self._settings["wheelSymmetryError"],
            self._settings["ignoreProximity"],
            self._settings["ignoreCliffSensors"],
            self._settings["ignoreIRSensors"],
            self._settings["ignoreFloorSensors"],
            self._settings["pluggedIn"],
        )
        self.serialInterface.SendToPropellerOverSerial("settings", settingsData, False)

    def _propellerConfigDataHandler(self, data):
        self.screen.addLine(str(data))
        if len(data) > 7:  # Ignore short packets
            # Round these to the same precision as the input was given at
            self._config["trackWidth"] = round(
                data[0], len(str(self._settings["trackWidth"]).split(".")[1])
            )
            self._config["distancePerCount"] = round(
                data[1], len(str(self._settings["distancePerCount"]).split(".")[1])
            )
            self._config["wheelSymmetryError"] = round(
                data[2], len(str(self._settings["wheelSymmetryError"]).split(".")[1])
            )
            self._config["ignoreProximity"] = data[3]
            self._config["ignoreCliffSensors"] = data[4]
            self._config["ignoreIRSensors"] = data[5]
            self._config["ignoreFloorSensors"] = data[6]
            self._config["pluggedIn"] = data[7]

    # noinspection Duplicates
    def _propellerOdomDataHandler(self, data):
        # This is only intended for testing. Your calling script should provide
        # an odomResponseFunction to deal with incoming odometry data.

        # self.screen.addLine("o: Odom Packet Received:" + str(data))

        if len(data) > 16:  # Ignore short packets
            # Fill Telemetry dictionary with data.
            self._telemetry["X"] = data[0]
            self._telemetry["Y"] = data[1]
            self._telemetry["Heading"] = data[2]
            self._telemetry["gyroHeading"] = data[3]
            self._telemetry["V"] = data[4]
            self._telemetry["Omega"] = data[5]
            self._telemetry["leftMotorPower"] = data[6]
            self._telemetry["rightMotorPower"] = data[7]
            self._telemetry["abd_speedLimit"] = data[8]
            self._telemetry["abdR_speedLimit"] = data[9]
            self._telemetry["minDistanceSensor"] = data[10]
            self._telemetry["safeToProceed"] = data[11]
            self._telemetry["safeToRecede"] = data[12]
            self._telemetry["Escaping"] = data[13]
            self._telemetry["cliff"] = data[14]
            self._telemetry["floorO"] = data[15]

            # Check if any maneuvers are in progress and keep them going or stop when done
            if self._navigating_distance or self._navigating_rotation:
                self.checkManeuverProgress()

            # For debugging exactly what the difference was
            """
            if self._telemetry['trackWidth'] != self._settings['trackWidth']:
                self.screen.addLine(
                    str(self._telemetry['trackWidth']) + ' ' + str(self._settings['trackWidth']))
            if self._telemetry['distancePerCount'] != self._settings['distancePerCount']:
                self.screen.addLine(
                    str(self._telemetry['distancePerCount']) + ' ' + str(self._settings['distancePerCount']))
            if self._telemetry['ignoreProximity'] != self._settings['ignoreProximity']:
                self.screen.addLine(
                    str(self._telemetry['ignoreProximity']) + ' ' + str(self._settings['ignoreProximity']))
            if self._telemetry['ignoreCliffSensors'] != self._settings['ignoreCliffSensors']:
                self.screen.addLine(str(self._telemetry['ignoreCliffSensor']s) + ' ' + str(
                    self._settings['ignoreCliffSensors']))
            if self._telemetry['ignoreIRSensors'] != self._settings['ignoreIRSensors']:
                self.screen.addLine(
                    str(self._telemetry['ignoreIRSensors']) + ' ' + str(self._settings['ignoreIRSensors']))
            if self._telemetry['ignoreFloorSensors'] != self._settings['ignoreFloorSensors']:
                self.screen.addLine(str(self._telemetry['ignoreFloorSensor']s) + ' ' + str(
                    self._settings['ignoreFloorSensors']))
            if self._telemetry['pluggedIn'] != self._settings['pluggedIn']:
                self.screen.addLine(
                    str(self._telemetry['pluggedIn']) + ' ' + str(self._settings['pluggedIn']))
            """

            # Fill sensory arrays with sensor data
            start = 16
            end = start + self.serialInterface.pingCount
            telemetry_pingData = data[start:end]
            start = end
            end = start + self.serialInterface.irCount
            telemetry_irData = data[start:end]
            start = end
            end = start + self.serialInterface.floorSensorCount
            telemetry_floorSensorData = data[start:end]
            start = end
            end = start + self.serialInterface.buttonCount
            telemetry_buttonInputData = data[start:end]
            start = end
            end = start + self.serialInterface.ledCount
            telemetry_ledInputData = data[start:end]

            # Update LED status
            self._telemetry["ledStatus"] = list(telemetry_ledInputData)
            # The first time, the self._settings version won't have any data in it
            if len(self._settings["ledStatus"]) < len(self._telemetry["ledStatus"]):
                self._settings["ledStatus"] = list(self._telemetry["ledStatus"])

            # Check if LEDs need to be updated
            if self._telemetry["ledStatus"] != self._settings["ledStatus"]:
                self._ledUpdateRequired = True

            # Create lines used to update data on Screen
            odomLineOne = (
                "Telemetry: X:"
                + format(self._telemetry["X"], ".2f").rjust(5)
                + " | Y:"
                + format(self._telemetry["Y"], ".2f").rjust(5)
                + " | Heading:"
                + format(self._telemetry["Heading"], ".2f").rjust(5)
                + " | Gyro:"
                + format(self._telemetry["gyroHeading"], ".2f").rjust(5)
                + " | V:"
                + format(self._telemetry["V"], ".2f").rjust(5)
                + " | Omega:"
                + format(self._telemetry["Omega"], ".2f").rjust(5)
                + " | leftPwr:"
                + format(self._telemetry["leftMotorPower"], ".2f")
                + " | rightPwr:"
                + format(self._telemetry["rightMotorPower"], ".2f")
            )
            odomLineTwo = "Settings: "
            odomLineTwo = (
                odomLineTwo
                + "pluggedIn:"
                + ("Yes", "No")[self._config["pluggedIn"] == 0]
            )
            odomLineTwo = (
                odomLineTwo
                + " | ignoreAllProximitySensors:"
                + ("Yes", "No")[self._config["ignoreProximity"] == 0]
            )
            odomLineTwo = (
                odomLineTwo
                + " | ignoreCliffSensors:"
                + ("Yes", "No")[self._config["ignoreCliffSensors"] == 0]
            )
            odomLineTwo = (
                odomLineTwo
                + " | ignoreIRSensors:"
                + ("Yes", "No")[self._config["ignoreIRSensors"] == 0]
            )
            odomLineTwo = (
                odomLineTwo
                + " | ignoreFloorSensors:"
                + ("Yes", "No")[self._config["ignoreFloorSensors"] == 0]
            )
            # These should already be rounded during input recording
            odomLineTwo = (
                odomLineTwo + " | trackWidth:" + str(self._config["trackWidth"])
            )
            odomLineTwo = (
                odomLineTwo
                + " | distancePerCount:"
                + str(self._config["distancePerCount"])
            )
            odomLineTwo = (
                odomLineTwo
                + " | wheelSymmetryError:"
                + str(self._config["wheelSymmetryError"])
            )
            odomLineThree = (
                "Status: abd_speedLimit:"
                + str(self._telemetry["abd_speedLimit"]).rjust(3)
                + " | abdR_speedLimit:"
                + str(self._telemetry["abdR_speedLimit"]).rjust(3)
                + " | minDistanceSensor:"
                + str(self._telemetry["minDistanceSensor"]).rjust(2)
                + " | safeToProceed:"
                + ("False", "True")[self._telemetry["safeToProceed"]]
                + " | safeToRecede:"
                + ("False", "True")[self._telemetry["safeToRecede"]]
                + " | Escaping:"
                + ("False", "True")[self._telemetry["Escaping"]]
            )
            odomLineFour = (
                "Sensors: Cliff Detected:"
                + ("No", "Yes")[self._telemetry["cliff"]]
                + " | Floor Obstacle Detected:"
                + ("No", "Yes")[self._telemetry["floorO"]]
            )
            odomLineFive = "PING"
            for entry in telemetry_pingData:
                odomLineFive = odomLineFive + str(entry).rjust(4)
            odomLineFive = odomLineFive + " | IR"
            for entry in telemetry_irData:
                odomLineFive = odomLineFive + str(entry).rjust(4)
            odomLineFive = odomLineFive + " | FLR"
            for entry in telemetry_floorSensorData:
                odomLineFive = odomLineFive + str(entry).rjust(2)
            odomLineFive = odomLineFive + " | BTN"
            for entry in telemetry_buttonInputData:
                odomLineFive = odomLineFive + str(entry).rjust(2)
            odomLineFive = odomLineFive + " | LED"
            for entry in telemetry_ledInputData:
                odomLineFive = odomLineFive + str(entry).rjust(2)

            # Update data on Screen
            self.screen.setOdomLines(
                odomLineOne, odomLineTwo, odomLineThree, odomLineFour, odomLineFive
            )

        # This code was just used for testing/debugging:
        # self.screen.addLine('Odom:')
        # end = 6
        # self.screen.addLine(data[:end])
        # start = end
        # end = start + 7
        # self.screen.addLine(data[start:end])
        # start = end
        # end = start + 10
        # self.screen.addLine(data[start:end])
        # start = end
        # end = start + self.serialInterface.pingCount
        # self.screen.addLine(data[start:end])
        # start = end
        # end = start + self.serialInterface.irCount
        # self.screen.addLine(data[start:end])
        # start = end
        # end = start + self.serialInterface.floorSensorCount
        # self.screen.addLine(data[start:end])
        # # start = end
        # # end = start + self.buttonCount
        # # self.screen.addLine()(data[start:end])
        # start = end
        # end = len(data)
        # self.screen.addLine(data[start:end])
        # self.screen.addLine('-----------------------------------')

    def TestDataResponseFunction(self, data):
        self.testPacketsReceived += 1
        if self._printIncomingTestPackets:
            self.screen.addLine("i: " + str(data))

    def CreateTestBadDataPacket(self, title):
        self.screen.addLine("-----------------------------------")
        self.screen.addLine(title + ":")
        self.testByte += 1
        if self.testByte > 255:
            self.testByte = 0
        badData = struct.pack(
            self.dataTypes.FormatTest,
            self.testUnsignedLong,
            self.testIntOne,
            self.testIntTwo,
            self.testIntThree,
            self.testByte,
            self.testCharacter,
            self.testFloat,
        )
        self.screen.addLine(
            "o: " + str(struct.unpack(self.dataTypes.FormatTest, badData))
        )
        return badData

    def SendGoodDataTestPacket(self):
        self.screen.addLine("-----------------------------------")
        self.screen.addLine("Good data:")
        self.testByte += 1
        if self.testByte > 255:
            self.testByte = 0
        self.testData = self.dataTypes.TestDataPacket(
            self.testUnsignedLong,
            self.testIntOne,
            self.testIntTwo,
            self.testIntThree,
            self.testByte,
            self.testCharacter,
            self.testFloat,
        )
        self.serialInterface.SendToPropellerOverSerial("test", self.testData, True)
        time.sleep(0.1)

    # We DO call protected members in this code,
    # because it is directly testing an interface,
    # that is normally never called directly,
    # in order to force various errors.
    # noinspection PyProtectedMember
    # noinspection Duplicates
    def _genericSerialTest(self):
        # NOTE: Test functions below perform operations that should not normally be carried out directly, like calculating checksum and building data packets
        self.screen.addLine("-----------------------------------")
        self.screen.addLine("Starting General Serial Test")
        numLoops = 0
        while numLoops < 10:
            self.SendGoodDataTestPacket()
            numLoops += 1

        # Test some bad data writes

        self.screen.addLine("-----------------------------------")
        self.screen.addLine(
            "Good start/end markers, but packet is missing an expected entry:"
        )
        badData = struct.pack(
            "<LhhhBc",
            self.testUnsignedLong,
            self.testIntOne,
            self.testIntTwo,
            self.testIntThree,
            self.testByte,
            self.testCharacter,
        )
        self.screen.addLine("o: " + str(struct.unpack("<LhhhBc", badData)))
        badData = self.serialInterface._AddCheckSum(badData)
        badData = self.serialInterface._EncodeHighBytes(badData)
        txLen = len(badData)
        dataToSend = bytearray()
        dataToSend.append(self.serialInterface._serialDataStartMarker)
        dataToSend.append(txLen)
        dataToSend.append(ord("t"))
        dataToSend.extend(badData)
        dataToSend.append(self.serialInterface._serialDataEndMarker)
        self.serialInterface._SerialDataGateway.Write(dataToSend)
        time.sleep(0.3)

        self.SendGoodDataTestPacket()

        self.screen.addLine("-----------------------------------")
        self.screen.addLine("Good start/end markers, but packet has extra data:")
        badData = struct.pack(
            "<LhhhBcfB",
            self.testUnsignedLong,
            self.testIntOne,
            self.testIntTwo,
            self.testIntThree,
            self.testByte,
            self.testCharacter,
            self.testFloat,
            128,
        )
        self.screen.addLine("o: " + str(struct.unpack("<LhhhBcfB", badData)))
        badData = self.serialInterface._AddCheckSum(badData)
        badData = self.serialInterface._EncodeHighBytes(badData)
        txLen = len(badData)
        dataToSend = bytearray()
        dataToSend.append(self.serialInterface._serialDataStartMarker)
        dataToSend.append(txLen)
        dataToSend.append(ord("t"))
        dataToSend.extend(badData)
        dataToSend.append(self.serialInterface._serialDataEndMarker)
        self.serialInterface._SerialDataGateway.Write(dataToSend)
        time.sleep(0.1)

        self.SendGoodDataTestPacket()

        badData = self.CreateTestBadDataPacket("Bad start marker")
        badData = self.serialInterface._AddCheckSum(badData)
        badData = self.serialInterface._EncodeHighBytes(badData)
        txLen = len(badData)
        dataToSend = bytearray()
        dataToSend.append(22)
        dataToSend.append(txLen)
        dataToSend.append(ord("t"))
        dataToSend.extend(badData)
        dataToSend.append(self.serialInterface._serialDataEndMarker)
        self.serialInterface._SerialDataGateway.Write(dataToSend)
        time.sleep(0.1)

        self.SendGoodDataTestPacket()

        badData = self.CreateTestBadDataPacket("Bad end marker")
        badData = self.serialInterface._AddCheckSum(badData)
        badData = self.serialInterface._EncodeHighBytes(badData)
        txLen = len(badData)
        dataToSend = bytearray()
        dataToSend.append(self.serialInterface._serialDataStartMarker)
        dataToSend.append(txLen)
        dataToSend.append(ord("t"))
        dataToSend.extend(badData)
        dataToSend.append(251)
        self.serialInterface._SerialDataGateway.Write(dataToSend)
        time.sleep(0.1)

        self.SendGoodDataTestPacket()
        self.SendGoodDataTestPacket()

        badData = self.CreateTestBadDataPacket("No start marker")
        badData = self.serialInterface._AddCheckSum(badData)
        badData = self.serialInterface._EncodeHighBytes(badData)
        txLen = len(badData)
        dataToSend.append(txLen)
        dataToSend.append(ord("t"))
        dataToSend.extend(badData)
        dataToSend.append(self.serialInterface._serialDataEndMarker)
        self.serialInterface._SerialDataGateway.Write(dataToSend)
        time.sleep(0.1)

        self.SendGoodDataTestPacket()

        badData = self.CreateTestBadDataPacket("Length shorter than we claim")
        badData = self.serialInterface._AddCheckSum(badData)
        badData = self.serialInterface._EncodeHighBytes(badData)
        txLen = len(badData) + 1
        dataToSend = bytearray()
        dataToSend.append(self.serialInterface._serialDataStartMarker)
        dataToSend.append(txLen)
        dataToSend.append(ord("t"))
        dataToSend.extend(badData)
        dataToSend.append(self.serialInterface._serialDataEndMarker)
        self.serialInterface._SerialDataGateway.Write(dataToSend)
        time.sleep(0.1)

        self.SendGoodDataTestPacket()

        badData = self.CreateTestBadDataPacket("Length longer than we claim")
        badData = self.serialInterface._AddCheckSum(badData)
        badData = self.serialInterface._EncodeHighBytes(badData)
        txLen = len(badData) - 1
        dataToSend = bytearray()
        dataToSend.append(+self.serialInterface._serialDataStartMarker)
        dataToSend.append(txLen)
        dataToSend.append(ord("t"))
        dataToSend.extend(badData)
        dataToSend.append(self.serialInterface._serialDataEndMarker)
        self.serialInterface._SerialDataGateway.Write(dataToSend)
        time.sleep(0.1)

        self.SendGoodDataTestPacket()

        badData = self.CreateTestBadDataPacket("Good data split into two writes, one")
        badData = self.serialInterface._AddCheckSum(badData)
        badData = self.serialInterface._EncodeHighBytes(badData)
        txLen = len(badData)
        dataToSend = bytearray()
        dataToSend.append(self.serialInterface._serialDataStartMarker)
        dataToSend.append(txLen)
        dataToSend.append(ord("t"))
        dataToSend.append(badData[0])
        dataToSend.append(badData[1])
        dataToSend.append(badData[2])
        dataToSend.append(badData[3])
        dataToSend.append(badData[4])
        dataToSend.append(badData[5])
        dataToSend.append(badData[6])
        dataToSend.append(badData[7])
        self.serialInterface._SerialDataGateway.Write(dataToSend)
        time.sleep(0.1)

        self.screen.addLine("-----------------------------------")
        self.screen.addLine("Good data split into two writes, two:")
        dataToSend = bytearray()
        dataToSend.append(badData[8])
        dataToSend.append(badData[9])
        dataToSend.append(badData[10])
        dataToSend.append(badData[11])
        dataToSend.append(badData[12])
        dataToSend.append(badData[13])
        dataToSend.append(badData[14])
        dataToSend.append(badData[15])
        dataToSend.append(badData[16])
        dataToSend.append(badData[17])
        dataToSend.append(badData[18])
        dataToSend.append(self.serialInterface._serialDataEndMarker)
        self.serialInterface._SerialDataGateway.Write(dataToSend)
        time.sleep(0.1)

        self.SendGoodDataTestPacket()

        badData = self.CreateTestBadDataPacket("Bad checksum")
        # Manually add bogus checksum to packet
        badData = badData + six.int2byte(24)
        badData = self.serialInterface._EncodeHighBytes(badData)
        txLen = len(badData)
        dataToSend = bytearray()
        dataToSend.append(self.serialInterface._serialDataStartMarker)
        dataToSend.append(txLen)
        dataToSend.append(ord("t"))
        dataToSend.extend(badData)
        dataToSend.append(self.serialInterface._serialDataEndMarker)
        self.serialInterface._SerialDataGateway.Write(dataToSend)
        time.sleep(0.1)

        self.SendGoodDataTestPacket()

        self.SendGoodDataTestPacket()

        self.SendGoodDataTestPacket()

        self.SendGoodDataTestPacket()

        self.SendGoodDataTestPacket()

        self.screen.addLine("-----------------------------------")
        self.screen.addLine("General Serial Test Complete")

    def _speedTestRawBps(self):
        numLoops = 0
        self.screen.addLine("Making sure Activity Board is ready...")
        totalDataSent = 0
        self.screen.addLine("Starting Serial Speed Test")
        # time in seconds since the epoch as a floating point number.
        startTime = time.time()
        self._printIncomingTestPackets = False
        self._printOutgoingTestPackets = False
        while numLoops < 500000:
            sentDataLength = self.serialInterface.SendToPropellerOverSerial(
                "test", self.testData, self._printOutgoingTestPackets
            )
            elapsedTime = time.time() - startTime
            sentDataLength = sentDataLength * 8  # Convert bytes to bits
            totalDataSent += sentDataLength
            self.screen.setStatusLine(
                format((totalDataSent * 8) / elapsedTime, ".2f")
                + " TOTAL bits per second SENT for "
                + str(totalDataSent)
                + " bits over "
                + format(elapsedTime, ".2f")
                + " seconds"
            )
            self.testPacketsSent += 1
            # 87975.2481965 TOTAL bits per second SENT for 88000000 bits over 8002.25079703 seconds
            # Done
            # TODO: This ignores the incoming bits
            numLoops += 1
            if self._lastInputCommand == "quit":
                break
            if self._lastInputCommand == "interrupt":
                # noinspection PyStatementEffect
                self._lastInputCommand == "Received"
                break
            while self._stallForInit:
                time.sleep(1)
        self._printIncomingTestPackets = True
        self._printOutgoingTestPackets = True
        self.screen.addLine("Serial Speed Test Complete")

    def sendMoveCommands(self, key):
        # Essentially copied from
        # https://github.com/ros-teleop/teleop_twist_keyboard/blob/master/teleop_twist_keyboard.py

        moveBindings = {
            "stop": (0, 0, 0, 0),
            "i": (1, 0, 0, 0),
            "o": (1, 0, 0, -1),
            "j": (0, 0, 0, 1),
            "k": (0, 0, 0, 0),
            "l": (0, 0, 0, -1),
            "u": (1, 0, 0, 1),
            ",": (-1, 0, 0, 0),
            ".": (-1, 0, 0, 1),
            "m": (-1, 0, 0, -1),
            "O": (1, -1, 0, 0),
            "I": (1, 0, 0, 0),
            "J": (0, 1, 0, 0),
            "K": (0, 0, 0, 0),
            "L": (0, -1, 0, 0),
            "U": (1, 1, 0, 0),
            "<": (-1, 0, 0, 0),
            ">": (-1, -1, 0, 0),
            "M": (-1, 1, 0, 0),
            "t": (0, 0, 1, 0),
            "b": (0, 0, -1, 0),
        }

        speedBindings = {
            "a": (1.1, 1.1),
            "z": (0.9, 0.9),
            "s": (1.1, 1),
            "x": (0.9, 1),
            "d": (1, 1.1),
            "c": (1, 0.9),
        }

        if key in moveBindings.keys():
            x = moveBindings[key][0]
            y = moveBindings[key][1]
            z = moveBindings[key][2]
            th = moveBindings[key][3]

            twist = Twist
            twist.linear.x = x * self.last_move_speed
            twist.linear.y = y * self.last_move_speed
            twist.linear.z = z * self.last_move_speed
            twist.angular.x = 0
            twist.angular.y = 0
            twist.angular.z = th * self.last_move_turn

            # All the robot (Propeller Board) actually needs is the
            # twist.linear.x and the twist.angular.z
            # The others aren't directions it can go.

            # Note command in scrolling log for debugging
            self.screen.addLine(
                "Move key "
                + str(key)
                + " received: "
                + str(twist.linear.x)
                + " "
                + str(twist.angular.z)
            )

            # Create packet for robot
            moveData = self.dataTypes.MoveDataPacket(twist.linear.x, twist.angular.z)

            # "Publish" command to robot
            self.serialInterface.SendToPropellerOverSerial("move", moveData, True)

        elif key in speedBindings.keys():
            self.last_move_speed = self.last_move_speed * speedBindings[key][0]
            self.last_move_turn = self.last_move_turn * speedBindings[key][1]
            # Note command in scrolling log for debugging
            self.screen.addLine(
                "Move key "
                + str(key)
                + " received. New Move speed: Linear: "
                + str(self.last_move_speed)
                + " Angular: "
                + str(self.last_move_turn)
            )

    def sameSign(self, num1, num2):
        return num1 >= 0 and num2 >= 0 or num1 < 0 and num2 < 0

    def smallestSignedAngleBetween(self, x, y):
        TAU = 2 * pi
        a = (x - y) % TAU
        b = (y - x) % TAU
        return -a if a < b else b

    def checkManeuverProgress(self):
        if self._navigating_distance:
            distance = sqrt(
                pow((self._telemetry["X"] - self._previous_position["X"]), 2)
                + pow((self._telemetry["Y"] - self._previous_position["Y"]), 2)
            )
            self.screen.addLine("Distance Traveled: " + str(distance) + " meters")
            if distance >= self._goal_distance:
                self.screen.addLine("Maneuver complete.")
                self.performManeuvers("maneuvers_cancel")
            else:
                self.sendMoveCommands(self._current_maneuver_command)
        elif self._navigating_rotation:
            angle_difference = self.smallestSignedAngleBetween(
                self._telemetry["Heading"], self._goal_rotation
            )
            self.screen.addLine(
                "Hed: "
                + str(self._telemetry["Heading"])
                + " Goal: "
                + str(self._goal_rotation)
                + " Traveled: "
                + str(angle_difference - self._previous_difference)
                + " Remaining: "
                # str(self._previous_heading)
                # + " "
                # + " "
                + str(angle_difference)
                + " rad"
                # + " "
                # + str(self._goal_rotation)
            )
            if self._previous_difference == 0.0:
                # Initial setting after rotation starts
                self._previous_difference = angle_difference
            # Check that the difference is sufficiently small (it will never be 0)
            # ************************************************************************* #
            # You can use this calculator: https://www.omnicalculator.com/math/arc-length
            # use the trackWidth (from arlobot.yaml) for the Diameter,
            # and the distancePerCount for the Arc Length
            # Central Angle is the minimum measurement resolution.
            # although you may get half that intermittently as one wheel moves,
            # and then the other, sort of "wiggling" to the next full point,
            # (Remember, if only one wheel has moved, we aren't exactly in the circle,
            # until the other one catches up)
            # For driveGeometry: {trackWidth: 0.403, distancePerCount: 0.00338}
            # resolution is 0.0167742 rad or 0.96109 deg
            # ************************************************************************* #

            # or if it has crossed, changing signs, however the sign is often wrong at the
            # beginning of the maneuver, so only check for sign changes when we are close to done.
            if abs(angle_difference) < 0.0167742 or (
                abs(angle_difference) < 1
                and not self.sameSign(angle_difference, self._previous_difference)
            ):
                self.screen.addLine("Maneuver complete.")
                self.performManeuvers("maneuvers_cancel")
            else:
                start_slow_down_angle = 0.2  # rad
                if abs(angle_difference) < start_slow_down_angle:
                    if self._saved_move_turn == 0.0:
                        self._saved_move_turn = self.last_move_turn
                    # Slow down as we approach the goal
                    new_move_speed = (
                        (self._saved_move_turn - 0) / (start_slow_down_angle - 0.01)
                    ) * abs(angle_difference) - 0.01
                    if new_move_speed > 0.06:
                        self.last_move_turn = new_move_speed
                self.sendMoveCommands(self._current_maneuver_command)
                self._previous_difference = angle_difference

    def performManeuvers(self, inputCommand):
        command = inputCommand.split("_")[1]
        self.screen.addLine("Maneuver: " + str(command))

        if command == "cancel":
            self._navigating_distance = False
            self._navigating_rotation = False
            self._goal_distance = 0.0
            self._goal_rotation = 0.0
            self._previous_difference = 0.0
            if not self._saved_move_turn == 0.0:
                self.last_move_turn = self._saved_move_turn
            self._previous_position["X"] = self._telemetry["X"]
            self._previous_position["Y"] = self._telemetry["Y"]
            self.sendMoveCommands("stop")
            self._current_maneuver_command = "stop"
        elif command == "reverse":
            self._maneuver_rotation_reversed = not self._maneuver_rotation_reversed
        elif command == "rotate":
            rotationAmount = inputCommand.split("_")[2]
            self._current_maneuver_command = "j"
            if self._maneuver_rotation_reversed:
                self._current_maneuver_command = "l"
            self._previous_heading = self._telemetry["Heading"]
            self._navigating_rotation = True
            goal_rotation = self._telemetry["Heading"] + radians(int(rotationAmount))
            if self._maneuver_rotation_reversed:
                goal_rotation = self._telemetry["Heading"] - radians(
                    int(rotationAmount)
                )
            if goal_rotation > pi:
                goal_rotation -= 2 * pi
            if goal_rotation < -pi:
                goal_rotation += 2 * pi
            self._goal_rotation = goal_rotation
            self.sendMoveCommands(self._current_maneuver_command)
        elif command == "forward":
            self._previous_position["X"] = self._telemetry["X"]
            self._previous_position["Y"] = self._telemetry["Y"]
            self._navigating_distance = True
            self._goal_distance = 1.0  # TODO: make this configurable
            self.sendMoveCommands("i")
            self._current_maneuver_command = "i"
        elif command == "backward":
            self._previous_position["X"] = self._telemetry["X"]
            self._previous_position["Y"] = self._telemetry["Y"]
            self._navigating_distance = True
            self._goal_distance = 1.0  # TODO: make this configurable
            self.sendMoveCommands(",")
            self._current_maneuver_command = ","

    def sendSettingsUpdate(self):
        self._settingsUpdateRequired = False
        self.screen.addLine("Updating settings.")
        settingsData = self.dataTypes.SettingsDataPacket(
            self._settings["trackWidth"],
            self._settings["distancePerCount"],
            self._settings["wheelSymmetryError"],
            self._settings["ignoreProximity"],
            self._settings["ignoreCliffSensors"],
            self._settings["ignoreIRSensors"],
            self._settings["ignoreFloorSensors"],
            self._settings["pluggedIn"],
        )
        self.serialInterface.SendToPropellerOverSerial("settings", settingsData, True)

    def _updateSetting(self, command):
        self.screen.addLine("Settings command " + str(command) + " received.")

        if command[0:13] == "abdSpeedLimit":
            if command.endswith("Up"):
                self._updateAbdSpeedLimit(
                    self._telemetry["abd_speedLimit"] + 1,
                    self._telemetry["abdR_speedLimit"],
                )
            else:
                self._updateAbdSpeedLimit(
                    self._telemetry["abd_speedLimit"] - 1,
                    self._telemetry["abdR_speedLimit"],
                )
        elif command[0:14] == "abdRSpeedLimit":
            if command.endswith("Up"):
                self._updateAbdSpeedLimit(
                    self._telemetry["abd_speedLimit"],
                    self._telemetry["abdR_speedLimit"] + 1,
                )
            else:
                self._updateAbdSpeedLimit(
                    self._telemetry["abd_speedLimit"],
                    self._telemetry["abdR_speedLimit"] - 1,
                )
        elif command == "trackWidthDown":
            self._settings["trackWidth"] = self._settings["trackWidth"] - 0.001
        elif command == "trackWidthUp":
            self._settings["trackWidth"] = self._settings["trackWidth"] + 0.001
        elif command == "distancePerCountDown":
            self._settings["distancePerCount"] = (
                self._settings["distancePerCount"] - 0.00001
            )
        elif command == "distancePerCountUp":
            self._settings["distancePerCount"] = (
                self._settings["distancePerCount"] + 0.00001
            )
        elif command == "wheelSymmetryErrorDown":
            self._settings["wheelSymmetryError"] = (
                self._settings["wheelSymmetryError"] - 0.01
            )
        elif command == "wheelSymmetryErrorUp":
            self._settings["wheelSymmetryError"] = (
                self._settings["wheelSymmetryError"] + 0.01
            )
        elif command == "ignoreProximity":
            self._settings["ignoreProximity"] = (1, 0)[
                self._settings["ignoreProximity"] == 1
            ]
        elif command == "ignoreCliffSensors":
            self._settings["ignoreCliffSensors"] = (1, 0)[
                self._settings["ignoreCliffSensors"] == 1
            ]
        elif command == "ignoreIRSensors":
            self._settings["ignoreIRSensors"] = (1, 0)[
                self._settings["ignoreIRSensors"] == 1
            ]
        elif command == "ignoreFloorSensors":
            self._settings["ignoreFloorSensors"] = (1, 0)[
                self._settings["ignoreFloorSensors"] == 1
            ]
        elif command == "pluggedIn":
            self._settings["pluggedIn"] = (1, 0)[self._settings["pluggedIn"] == 1]

    def _updateAbdSpeedLimit(self, abd_speedLimit, abdR_speedLimit):
        self.screen.addLine(
            "Updating abd_speedLimit "
            + str(abd_speedLimit)
            + " "
            + "abdR_speedLimit "
            + str(abdR_speedLimit)
        )
        abdOverrideData = self.dataTypes.AbdParameterOverrideDataPacket(
            abd_speedLimit, abdR_speedLimit
        )
        self.serialInterface.SendToPropellerOverSerial(
            "abdOverride", abdOverrideData, True
        )

    def _updateLED(self, led_number):
        if len(self._settings["ledStatus"]) > led_number:
            # In case ledStatus array isn't initialized yet
            self._settings["ledStatus"][led_number] = (
                1 - self._settings["ledStatus"][led_number]
            )

    def _sendLedUpdate(self):
        self._ledUpdateRequired = False
        for i in range(0, len(self._settings["ledStatus"])):
            if (
                len(self._telemetry["ledStatus"]) > i
                and self._settings["ledStatus"][i] != self._telemetry["ledStatus"][i]
            ):
                self.screen.addLine("Updating LED " + str(i))
                ledData = self.dataTypes.LEDDataPacket(
                    i, self._settings["ledStatus"][i]
                )
                self.serialInterface.SendToPropellerOverSerial("led", ledData, True)

    def _sendPositionUpdate(self, entry, data):
        self.screen.addLine("Position Update: " + str(entry) + " = " + str(data))
        x = self._telemetry["X"]
        y = self._telemetry["Y"]
        heading = self._telemetry["Heading"]
        if entry == "X":
            x = float(data)
        elif entry == "Y":
            y = float(data)
        elif entry == "Heading":
            heading = float(data)
        data = self.dataTypes.PositionUpdateDataPacket(x, y, heading)
        self.serialInterface.SendToPropellerOverSerial("positionUpdate", data, True)

    # noinspection Duplicates
    def run(self):
        t = threading.Thread(target=curses.wrapper, args=(self.screen.run,))
        t.setDaemon(True)
        t.start()

        self.serialInterface.Start()

        while True:
            if self._lastInputCommand == "genericTest":
                self._lastInputCommand = "Received"
                self._genericSerialTest()
            elif self._lastInputCommand == "speedTestRawBps":
                self._lastInputCommand = "Received"
                self._speedTestRawBps()
            elif self._lastInputCommand == "togglePluggedIn":
                self._lastInputCommand = "Received"
                self._settings["pluggedIn"] = (1, 0)[self._settings["pluggedIn"]]
                self.TestSendSettingsFunction()
            elif self._lastInputCommand[0:4] == "move":
                inputLetter = self._lastInputCommand[4]
                self._lastInputCommand = "Received"
                self.sendMoveCommands(inputLetter)
            elif self._lastInputCommand[0:8] == "settings":
                inputCommand = self._lastInputCommand.split("_")[1]
                self._lastInputCommand = "Received"
                self._updateSetting(inputCommand)
            elif self._lastInputCommand[0:3] == "led":
                led_number = int(self._lastInputCommand.split("_")[1])
                self._lastInputCommand = "Received"
                self._updateLED(led_number)
            elif self._lastInputCommand[0:9] == "maneuvers":
                inputCommand = self._lastInputCommand
                self._lastInputCommand = "Received"
                self.performManeuvers(inputCommand)
            elif self._lastInputCommand.split("_")[0] == "overridePosition":
                inputCommand = self._lastInputCommand.split("_")
                self._lastInputCommand = "Received"
                self._sendPositionUpdate(inputCommand[1], inputCommand[2])
            elif self._lastInputCommand == "quit":
                # No point in updating _lastInputCommand if we are supposed to quit.
                self.serialInterface.Stop()
                break
            elif self._settingsUpdateRequired:
                self.sendSettingsUpdate()
            elif self._ledUpdateRequired:
                self._sendLedUpdate()
            else:
                # Check if any settings need to be updated
                if (
                    self._config["trackWidth"] != self._settings["trackWidth"]
                    or self._config["distancePerCount"]
                    != self._settings["distancePerCount"]
                    or self._config["wheelSymmetryError"]
                    != self._settings["wheelSymmetryError"]
                    or self._config["ignoreProximity"]
                    != self._settings["ignoreProximity"]
                    or self._config["ignoreCliffSensors"]
                    != self._settings["ignoreCliffSensors"]
                    or self._config["ignoreIRSensors"]
                    != self._settings["ignoreIRSensors"]
                    or self._config["ignoreFloorSensors"]
                    != self._settings["ignoreFloorSensors"]
                    or self._config["pluggedIn"] != self._settings["pluggedIn"]
                ):
                    self._settingsUpdateRequired = True

                # self.screen.addLine(str('bob'))
                time.sleep(0.5)

        if self._closingMessageToDisplay != "":
            print("\n\n\n\n")
            print(self._closingMessageToDisplay)

        """
        TODO:
        3. Guides user through test routines:
        
        Still needs:
        Better formatting so we can see the data more clearly.
        Color code "good"/"bad" things like a Escaping, etc.
        Help screen to show what the options are.
        
        Build in tests for odometry,
        It could even have "burn in" tests to make shapes like squares and circles,
        for X count and then we could see the results.
        Make the squares back and forth too,
        lines,
        etc. for testing odometry over the course of time,
        and for testing the stability of the system.
        """


if __name__ == "__main__":
    if (len(sys.argv)) < 2:
        print(
            "You must provide the Propeller Board USB Port name on the command line like this:"
        )
        print(str(sys.argv[0]) + " /dev/ttyUSB0")
        print(
            "You can use the find_ActivityBoard.sh script to find it, or better yet, do not call this directly. Instead run PropellerSerialTest.sh from the scripts folder."
        )
        sys.exit(1)
    PropellerSerialTest(sys.argv[1])
