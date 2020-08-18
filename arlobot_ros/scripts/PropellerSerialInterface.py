#!/usr/bin/env python

# ======================================
# Propeller Serial Interface
# ======================================
"""
This code provides a consistent interface for sending
and receiving data to the Propeller board on
the Arlobot.

It requires the SerialDataGateway.py to do the actual
connecting to the Propeller board.

This code should be called by another program that decides
what functions to call when,
and accepts the data from the Propeller board.

Inspired by Robin2's 26 Aug 2014 post:
http://forum.arduino.cc/index.php?topic=263107.0
and
http://forum.arduino.cc/index.php?topic=225329
"""

from __future__ import print_function

import time
import struct

import six
from PropellerSerialGateway import PropellerSerialGateway
from PropellerSerialDataPacketTypes import PropellerSerialDataPacketTypes


def _printOutputFunction(data):
    print(data)


class PropellerSerialInterface(object):
    def __init__(
        self,
        readyResponseFunction,
        odomResponseFunction,
        configResponseFunction,
        testDataResponseFunction,
        printOutputFunction=_printOutputFunction,
        port="/dev/ttyUSB0",
        baud_rate=115200,
    ):
        # NOTE the user must ensure that the serial port and baud_rate are correct
        self._Port = port
        self._Baud_rate = baud_rate
        self._serialReceiveBuffer = bytearray()
        self._serialReceiveInProgress = False
        self._serialReceiveDataLength = 0
        self._serialReceiveDataType = 0
        self._serialReceiveError = False
        self._serialReceiveIndex = 0
        self._serialSendError = False
        self._serialWriteInProgress = False
        self._serialWriteTimeout = 1000  # multiplied by 0.001

        self._readyResponseFunction = readyResponseFunction
        self._odomResponseFunction = odomResponseFunction
        self._configResponseFunction = configResponseFunction
        self._testDataResponseFunction = testDataResponseFunction
        self._output = printOutputFunction

        self._DataTypes = PropellerSerialDataPacketTypes()

        self._SerialDataGateway = PropellerSerialGateway(
            self._Port, self._Baud_rate, self.HandleSerialDataInput, self._output
        )

        # Special markers - These should match defines in C code
        self._serialDataStartMarker = 254  # define START_MARKER 254
        self._serialDataEndMarker = 255  # define END_MARKER 255
        self._serialDataSpecialByte = 253  # define SPECIAL_BYTE 253
        self._maximumSerialDataLength = 252  # define MAXIMUM_BUFFER_SIZE 252
        # Currently a 253 ("special bit") in the length field indicates an error message
        # coming in from the controller.
        # We could add more "signal bits" if we want to cut the max length down more.

        # These values tweak the serial write automatic speed setting
        self._serialWriteMinimumDelay = 0.001
        self._serialWriteMaximumDelay = 0.1
        self._serialWriteDelayIncrement = 0.001
        self._goodPacketCount = 0
        self._goodPacketDelayBeforeAdjusting = 1
        self._goodPacketDelayMax = 30
        self._lastGoodWriteDelay = 0
        self._badPacketCount = 0
        self._badPacketsBeforeAdjusting = 1
        self._serialWriteDelay = self._serialWriteMinimumDelay

        # Ready Data Example: 30, 14, 8, 4, 4
        self.sensorDataCount = 0
        self.pingCount = 0
        self.irCount = 0
        self.floorSensorCount = 0
        self.buttonCount = 0
        self.ledCount = 0
        self.propellerCodeVersion = 0

    def Start(self):
        self._SerialDataGateway.Start()

    def Stop(self):
        self._SerialDataGateway.Stop()

    def SendToPropellerOverSerial(self, packetType, data, debug=False):
        waiting = 0
        while self._serialWriteInProgress:
            if waiting > self._serialWriteTimeout:
                self._output(
                    "Send Error: Timed out waiting for Serial Write to not be busy."
                )
                return False
            time.sleep(0.001)
            waiting = waiting + 1
        self._serialWriteInProgress = True
        if packetType == "test":
            dataType = "t".encode("ascii")
            dataToSend = self._PackDataPacketTest(data)
            if debug:
                self._output(
                    "o: " + str(struct.unpack(self._DataTypes.FormatTest, dataToSend))
                )
        elif packetType == "init":
            dataType = "i".encode("ascii")
            dataToSend = self._PackDataPacketInit(data)
            if debug:
                self._output(
                    "o: " + str(struct.unpack(self._DataTypes.FormatInit, dataToSend))
                )
        elif packetType == "move":
            dataType = "m".encode("ascii")
            dataToSend = self._PackDataPacketMove(data)
            if debug:
                self._output(
                    "o: " + str(struct.unpack(self._DataTypes.FormatMove, dataToSend))
                )
        elif packetType == "settings":
            dataType = "s".encode("ascii")
            dataToSend = self._PackDataPacketSettings(data)
            if debug:
                self._output(
                    "o: "
                    + str(struct.unpack(self._DataTypes.FormatSettings, dataToSend))
                )
        elif packetType == "led":
            dataType = "l".encode("ascii")
            dataToSend = self._PackDataFormatLED(data)
            if debug:
                self._output(
                    "o: " + str(struct.unpack(self._DataTypes.FormatLED, dataToSend))
                )
        elif packetType == "positionUpdate":
            dataType = "p".encode("ascii")
            dataToSend = self._PackDataFormatPositionUpdate(data)
            if debug:
                self._output(
                    "o: "
                    + str(
                        struct.unpack(self._DataTypes.FormatPositionUpdate, dataToSend)
                    )
                )
        elif packetType == "abdOverride":
            dataType = "a".encode("ascii")
            dataToSend = self._PackDataFormatAbdParameterOverride(data)
            if debug:
                self._output(
                    "o: "
                    + str(
                        struct.unpack(
                            self._DataTypes.FormatAbdParameterOverride, dataToSend
                        )
                    )
                )
        else:
            self._output("Send Error: Unknown data type")
            self._serialWriteInProgress = False
            return False
        dataToSend = self._AddCheckSum(dataToSend)
        dataToSend = self._EncodeHighBytes(dataToSend)
        if len(dataToSend) > self._maximumSerialDataLength:
            self._output("Send Error: Attempted to send more bytes than allowed")
            self._serialWriteInProgress = False
            return False
        txLen = len(dataToSend)
        dataToSend = self._EncodeFinalDataPacket(dataToSend, txLen, dataType)
        self._SerialDataGateway.Write(dataToSend)

        # Serial write is complete, now wait for Propeller to be free,
        # before allowing further data to send
        readyToWrite = self._SerialDataGateway.PropellerReady()
        waiting = 0
        while not readyToWrite:
            if waiting > self._serialWriteTimeout:
                self._output(
                    "Send Error: Timed out waiting after Serial Write to not be busy."
                )
                return False
            time.sleep(self._serialWriteDelay)
            readyToWrite = self._SerialDataGateway.PropellerReady()
            waiting = waiting + 1
        self._serialWriteInProgress = False
        if debug:
            return len(dataToSend)
        return True

    def _EncodeHighBytes(self, inBytes):

        outBytes = bytearray()
        for element in bytearray(inBytes):
            if element >= self._serialDataSpecialByte:
                outBytes.append(self._serialDataSpecialByte)
                outBytes.append(element - self._serialDataSpecialByte)
            else:
                outBytes.append(element)

        return outBytes

    def _DecodeHighBytes(self, inBytes):

        outBytes = bytearray()
        decodeNextByte = False
        for element in bytearray(inBytes):
            if decodeNextByte:
                decodeNextByte = False
                if self._serialDataSpecialByte + element < 256:
                    outBytes.append(element + self._serialDataSpecialByte)
                else:
                    outBytes.append(0)
            else:
                if element == self._serialDataSpecialByte:
                    decodeNextByte = True
                else:
                    outBytes.append(element)

        return outBytes

    def HandleSerialDataInput(self, receivedByte):
        # This kind of mirrors PropellerCodeForArloBot/ReceiveSerialData.h
        if self._serialReceiveInProgress:
            if six.byte2int(receivedByte) != self._serialDataEndMarker:
                if (
                    self._serialReceiveDataLength == 0
                    and self._serialReceiveError is False
                ):
                    # First byte should be data length

                    self._serialReceiveDataLength = six.byte2int(receivedByte)
                    if (
                        self._serialReceiveDataLength < 1
                        or self._serialReceiveDataLength > self._maximumSerialDataLength
                    ):
                        # NOTE: For data FROM the Propeller, if this bit IS the Special Byte,
                        # then this is a text based error message. This means that this wil NOT be the data length.
                        if self._serialReceiveDataLength != self._serialDataSpecialByte:
                            self._output(
                                "Incoming Data Error: Invalid data length: "
                                + str(self._serialReceiveDataLength)
                            )
                            self._serialReceiveError = True
                elif (
                    self._serialReceiveDataType == 0
                    and self._serialReceiveError is False
                ):
                    self._serialReceiveDataType = receivedByte
                else:
                    # Check for long packet
                    # NOTE: Because we checked that dataLength is not longer than MAXIMUM_SERIAL_BUFFER_SIZE already,
                    # this check suffices to make sure we do not accept too much data
                    if self._serialReceiveIndex < self._serialReceiveDataLength:
                        self._serialReceiveBuffer = (
                            self._serialReceiveBuffer + receivedByte
                        )
                        self._serialReceiveIndex += 1
                    else:
                        if not self._serialReceiveError:
                            # We have to add 1 to receiveIndex, because that is what the data length WOULD be, if we added this bit,
                            # but we aren't adding it, we are just discarding the entire packet until we hit an end marker
                            # or another start marker.
                            self._output(
                                "Incoming Data Error: Data length "
                                + str(self._serialReceiveIndex + 1)
                                + " is longer than length bit "
                                + str(self._serialReceiveDataLength)
                            )
                        self._serialReceiveError = True
                        # We keep reading though and clearing all data until a valid closing marker.
            else:
                # End Marker received, validate, parse, and store data

                # Only use data if packet was not long
                if not self._serialReceiveError:
                    # For data FROM the Propeller, if the length bit IS the Special Byte,
                    # then this is a text based error message.
                    if self._serialReceiveDataLength == self._serialDataSpecialByte:
                        # NOTE: If you want to create specific responses to specific errors,
                        # this is where to do it. Typically though the best thing to do is
                        # just to send them to whatever displays or logs output.
                        # NOTE also that some "Errors" are simple status information
                        # that can be helpful in logging or debugging,
                        # like when the Propeller starts up or when the Odom cog
                        # is started.
                        try:
                            self._output(
                                "Received ERROR: "
                                + self._serialReceiveBuffer.decode("ascii")
                            )
                        except UnicodeError:
                            self._output("Received ERROR: Unable to decode.")
                        self._serialReceiveError = True
                    elif self._serialReceiveIndex < self._serialReceiveDataLength:
                        # Check for packets less than given length
                        self._output(
                            "Incoming Data Error: Data length "
                            + str(self._serialReceiveIndex)
                            + " does not match length bit "
                            + str(self._serialReceiveDataLength)
                        )
                        self._serialReceiveError = True

                if not self._serialReceiveError:
                    if len(self._serialReceiveBuffer) == 0:
                        self._serialReceiveError = True

                if not self._serialReceiveError:
                    # decode High Bytes
                    self._serialReceiveBuffer = self._DecodeHighBytes(
                        self._serialReceiveBuffer
                    )

                    # Checksum incoming data
                    # https://henryforceblog.wordpress.com/2015/03/12/designing-a-communication-protocol-using-arduinos-serial-library/
                    # The last element in the input is the checksum
                    receivedChecksum = self._serialReceiveBuffer[-1]

                    # You can use this to dump the packet data for debugging
                    # if things aren't working right with a new data type.
                    # self._output("Length: " + str(len(self._serialReceiveBuffer)))
                    # count = 0
                    # while count < len(self._serialReceiveBuffer):
                    #     position = 0
                    #     outputString = ''
                    #     while count < len(self._serialReceiveBuffer) > 0 and position < 10:
                    #         outputString = outputString + str(ord(self._serialReceiveBuffer[count])) + ', '
                    #         position += 1
                    #         count += 1
                    #     self._output(outputString)

                    # Remove the checksum from the data
                    self._serialReceiveBuffer = self._serialReceiveBuffer[:-1]
                    dataSum = 0
                    for element in self._serialReceiveBuffer:
                        dataSum = dataSum + element
                    dataCheckSum = dataSum & 0xFF

                    if receivedChecksum != dataCheckSum:
                        self._output(
                            "Incoming Data Error: Calculated checksum "
                            + str(dataCheckSum)
                            + " does not match received checksum "
                            + str(receivedChecksum)
                        )
                        self._serialReceiveError = True

                if not self._serialReceiveError:
                    if self._serialReceiveDataType == self._DataTypes.CharacterTest:
                        if len(self._serialReceiveBuffer) != struct.calcsize(
                            self._DataTypes.FormatTest
                        ):
                            self._serialReceiveError = True
                            self._output(
                                "Incoming Data Error: Incoming Test packet length "
                                + str(len(self._serialReceiveBuffer))
                                + " does not match expected packet length "
                                + str(struct.calcsize(self._DataTypes.FormatTest))
                                + "."
                            )
                        else:
                            data = struct.unpack(
                                self._DataTypes.FormatTest, self._serialReceiveBuffer
                            )
                            self._testDataResponseFunction(data)
                    elif self._serialReceiveDataType == self._DataTypes.CharacterReady:
                        if len(self._serialReceiveBuffer) != struct.calcsize(
                            self._DataTypes.FormatReady
                        ):
                            self._serialReceiveError = True
                            self._output(
                                "Incoming Data Error: Incoming Ready packet length "
                                + str(len(self._serialReceiveBuffer))
                                + " does not match expected packet length "
                                + str(struct.calcsize(self._DataTypes.FormatReady))
                                + "."
                            )
                        else:
                            data = struct.unpack(
                                self._DataTypes.FormatReady, self._serialReceiveBuffer
                            )

                            # Ready Data Example: 30, 14, 8, 4, 4
                            self.sensorDataCount = data[1]
                            self.pingCount = data[2]
                            self.irCount = data[3]
                            self.floorSensorCount = data[4]
                            self.buttonCount = data[5]
                            self.ledCount = data[6]
                            self.propellerCodeVersion = data[7]
                            self._DataTypes.setFormatOdom(self.sensorDataCount)

                            self._readyResponseFunction(data)
                    elif self._serialReceiveDataType == self._DataTypes.CharacterOdom:
                        if len(self._serialReceiveBuffer) != struct.calcsize(
                            self._DataTypes.FormatOdom
                        ):
                            self._serialReceiveError = True
                            self._output(
                                "Incoming Data Error: Incoming Odom packet length "
                                + str(len(self._serialReceiveBuffer))
                                + " does not match expected packet length "
                                + str(struct.calcsize(self._DataTypes.FormatOdom))
                                + "."
                            )
                        else:
                            data = struct.unpack(
                                self._DataTypes.FormatOdom, self._serialReceiveBuffer
                            )
                            self._odomResponseFunction(data)
                    elif self._serialReceiveDataType == self._DataTypes.CharacterConfig:
                        if len(self._serialReceiveBuffer) != struct.calcsize(
                            self._DataTypes.FormatConfig
                        ):
                            self._serialReceiveError = True
                            self._output(
                                "Incoming Data Error: Incoming Config packet length "
                                + str(len(self._serialReceiveBuffer))
                                + " does not match expected packet length "
                                + str(struct.calcsize(self._DataTypes.FormatConfig))
                                + "."
                            )
                        else:
                            data = struct.unpack(
                                self._DataTypes.FormatConfig, self._serialReceiveBuffer
                            )
                            self._configResponseFunction(data)

                """
                Speed up slowly, not instantly.
                Find a "working ceiling" and stay there, with only periodic attempts
                to go faster.
                """
                if self._serialReceiveError:
                    self._goodPacketCount = 0
                    self._badPacketCount += 1
                    if self._badPacketCount > self._badPacketsBeforeAdjusting:
                        self._badPacketCount = 0

                        if self._serialWriteDelay < self._serialWriteMaximumDelay:
                            self._serialWriteDelay += self._serialWriteDelayIncrement
                            self._output(
                                "Serial Write Delay: " + str(self._serialWriteDelay)
                            )

                            if self._serialWriteDelay == self._lastGoodWriteDelay:
                                if (
                                    self._goodPacketDelayBeforeAdjusting
                                    < self._goodPacketDelayMax
                                ):
                                    self._goodPacketDelayBeforeAdjusting += 1
                                    self._output(
                                        "Good Packet Delay: "
                                        + str(self._goodPacketDelayBeforeAdjusting)
                                    )
                            else:
                                if self._goodPacketDelayBeforeAdjusting > 1:
                                    self._goodPacketDelayBeforeAdjusting -= 1
                                    self._output(
                                        "Good Packet Delay: "
                                        + str(self._goodPacketDelayBeforeAdjusting)
                                    )

                elif self._serialWriteDelay > self._serialWriteMinimumDelay:
                    self._goodPacketCount += 1
                    if (
                        self._goodPacketCount * self._serialWriteDelay
                        > self._goodPacketDelayBeforeAdjusting
                    ):
                        self._goodPacketCount = 0
                        self._lastGoodWriteDelay = self._serialWriteDelay
                        self._serialWriteDelay -= self._serialWriteDelayIncrement
                        self._output(
                            "Serial Write Delay: " + str(self._serialWriteDelay)
                        )

                self._serialReceiveInProgress = False
                self._serialReceiveError = False

        elif receivedByte and six.byte2int(receivedByte) == self._serialDataStartMarker:
            self._serialReceiveInProgress = True
            self._serialReceiveDataLength = 0
            self._serialReceiveDataType = 0
            self._serialReceiveError = False
            self._serialReceiveIndex = 0
            self._serialReceiveBuffer = bytearray()
        else:
            if not self._serialReceiveError:
                self._output("Incoming Data Error: Data received outside of packet.")
            self._serialReceiveError = True

    def _AddCheckSum(self, inBytes):
        # https://henryforceblog.wordpress.com/2015/03/12/designing-a-communication-protocol-using-arduinos-serial-library/
        dataSum = 0
        for element in bytearray(inBytes):
            dataSum = dataSum + element
        dataCheckSum = dataSum & 0xFF
        outStr = inBytes + six.int2byte(dataCheckSum)

        return outStr

    def _EncodeFinalDataPacket(self, inBytes, txLen, dataType):
        outBytes = bytearray()
        outBytes.append(self._serialDataStartMarker)
        outBytes.append(txLen)
        outBytes.extend(dataType)
        outBytes.extend(inBytes)
        outBytes.append(self._serialDataEndMarker)
        return outBytes

    def _PackDataPacketTest(self, data):
        return struct.pack(
            self._DataTypes.FormatTest,
            data.testUnsignedLog,
            data.testIntOne,
            data.testIntTwo,
            data.testIntThree,
            data.testByte,
            data.testCharacter,
            data.testFloat,
        )

    def _PackDataPacketInit(self, data):
        return struct.pack(self._DataTypes.FormatInit, data.X, data.Y, data.Heading)

    def _PackDataPacketMove(self, data):
        return struct.pack(
            self._DataTypes.FormatMove,
            data.CommandedVelocity,
            data.CommandedAngularVelocity,
        )

    def _PackDataPacketSettings(self, data):
        return struct.pack(
            self._DataTypes.FormatSettings,
            data.trackWidth,
            data.distancePerCount,
            data.wheelSymmetryError,
            data.ignoreProximity,
            data.ignoreCliffSensors,
            data.ignoreIRSensors,
            data.ignoreFloorSensors,
            data.pluggedIn,
        )

    def _PackDataFormatLED(self, data):
        return struct.pack(self._DataTypes.FormatLED, data.ledNumber, data.ledState)

    def _PackDataFormatPositionUpdate(self, data):
        return struct.pack(
            self._DataTypes.FormatPositionUpdate, data.X, data.Y, data.Heading
        )

    def _PackDataFormatAbdParameterOverride(self, data):
        return struct.pack(
            self._DataTypes.FormatAbdParameterOverride,
            data.abd_speedLimit,
            data.abdR_speedLimit,
        )

    def _UnpackData(self, data):
        return struct.pack(
            self._DataTypes.FormatTest,
            data.testUnsignedLog,
            data.testIntOne,
            data.testIntTwo,
            data.testIntThree,
            data.testByte,
            data.testCharacter,
            data.testFloat,
        )


if __name__ == "__main__":
    print(
        "This script should be called from another script that will use it, not called directly. Try PropellerSerialTest.py if you want to test."
    )
