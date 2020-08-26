/*
This is the code to run on a Parallax Propeller based Activity Board
in order to interface ROS with an ArloBot.

Author: Chris L8 https://github.com/chrisl8
URL: https://github.com/chrisl8/ArloBot

The ROS Node for this code is called propellerbot_node.py
and can be found in the arlobot_ros package from the above URL.

ATTENTION! ATTENTION! ATTENTION! ATTENTION! ATTENTION! ATTENTION!

NOTE: This code is for the DHB-10 Motor Controller that comes with the new
Parallax Arlo kit. You MUST edit the settings in
~/.arlobot/per_robot_settings_for_propeller_c_code.h
based on the physical layout of your robot!
For each QUESTION:
UNCOMMENT '#define' lines for any included items,
COMMENT '#define' lines for anything that is not included.
For each SETTING:
Set the variable as required, noting that usually these are ignored if the
preceding QUESTION is commented out.

Example, My robot has a "Thing1", but not a "Thing2"
*/
#pragma clang diagnostic push
#pragma ide diagnostic ignored "EndlessLoop"
#define hasThingOne
//#define hasThingTwo

/* Just like that, comment out the "has" line for things you do not have,
and if you do have the thing, adjust the numbers on the other definition as
needed. By using the #define lines, code for items you do not have is never seen
by the compiler and is never even loaded on the Propeller board, saving memory
and avoiding any errors or crashes caused by unused code. */

#include "per_robot_settings_for_propeller_c_code.h"
/* If SimpleIDE build fails because the above file is missing,
open up the "Project Manager", then the "Compiler" tab,
and fix the path to your ~/.arlobot/ folder
under Other Compiler Options
and/or copy the above file from the dotfiles folder
to your ~/.arlobot folder.
You could also just move the files in the dotfiles folder into
the folder with this file, but future "git pull" updates
may erase your changes.

Full details on how to use the DHB-10 Motor Controller on the Parallax Arlo
Robot platform with a Propeller Activity Board can be found here:
http://learn.parallax.com/tutorials/robot/arlo/arlo-activity-board-brain
I highly suggets you work through the instructions there and run the example
programs and tests before using this code.

Special thanks to Dr. Rainer Hessmer. Much of this code is based on his work at
https://code.google.com/p/drh-robotics-ros/

Please also see these sites which helped me tremendously with the formulas:
http://www.seattlerobotics.org/encoder/200610/article3/IMU%20Odometry,%20by%20David%20Anderson.htm
http://webdelcire.com/wordpress/archives/527

And of course the entire point of this is to interface with ROS, so read about
everything ROS here: http://wiki.ros.org/

All code here heavily borrowed from everywhere code can be found! :)

SimpleIDE Options
Everything here is the default for a new SimpleIDE project except for "Enable
Pruning." Project Options: Board Type: ACTIVITYBOARD Compiler Type: C - This
is C code Memory Model: CMM Main RAM Compact - My code does not fit into LMM,
and there is no LMM version of arlodrive.h Optimization: -Os Size - When I
change this to "Speed" I get strange behavior. Compiler: CHECK - 32bit Double
CHECK - Enable Pruning - This does not make much difference, but a little.
Other Compiler Options: -std=c99 - this is part of the default SimpleIDE New
project Linker: CHECK - Math lib - required for floating point math! nothing
else checked or added under the Linker tab.

References:
https://forum.arduino.cc/index.php?topic=288234.0
https://forum.arduino.cc/index.php?topic=263107.0
https://forum.arduino.cc/index.php?topic=225329
https://forum.arduino.cc/index.php?topic=416340.0
*/

#include "versionNumber.h"

#include "fdserial.h"
#include "simpletools.h"
#include <stdbool.h> // This is how you get bool, true/false to work in c99!

#ifdef hasMCP3208
#include "mcp3208.h"
#endif

#ifdef hasQuickStartBoard
// PING Sensors are all handled by the QuickStart board if it is present.
#else
#include "ping.h"
#endif

#ifdef hasMotorPowerMonitorCircuit
/* Nothing else in my code uses the built in ADC,
although you could modify the code to use it with
IR sensors. */
// Include adcDCpropab
// http://learn.parallax.com/propeller-c-simple-circuits/measure-volts
#include "adcDCpropab.h"
#endif

/*
Full details on how to use the DHB-10 Motor Controller on the Parallax Arlo
Robot platform with a Propeller Activity Board can be found here:
http://learn.parallax.com/tutorials/robot/arlo/arlo-activity-board-brain
I highly suggest you work through the instructions there and run the example
programs and tests before using this code.
*/
#include "arlodrive.h"

#ifdef encoderConnectedToPropeller
// The DBH10 reads the encoder pins to give the wheel speed,
// however, some user have found the DBH10 to be unreliable, so they hook
// the encoder pins to the Propeller directly.
#include "encoderCount.h"
#endif

/* About when and why to use static and/or volatile variables on Propeller:
 https://learn.parallax.com/propeller-c-library-studies/functions-multiple-files-sharing-variables
 Ignore advice that is not directly about the Propeller CPU.
 */

// See ~/.arlobot/per_robot_settings_for_propeller_c_code.h to adjust
// MAXIMUM_SPEED
static int abd_speedLimit = MAXIMUM_SPEED;
// Reverse speed limit to allow robot to reverse fast if it is blocked in front
// and visa versa
static int abdR_speedLimit = MAXIMUM_SPEED;

fdserial *term;

// Robot description:
// We will get this from ROS so that it is easier to tweak between runs without
// reloading the Propeller EEPROM.
// http://learn.parallax.com/activitybot/calculating-angles-rotation
static float distancePerCount = 0.0;
static float trackWidth = 0.0;
static float wheelSymmetryError = 0.0;
/* See ~/.arlobot/arlobot.yaml
   to set or change these values
*/

// int adc_IR_cm(int); // Function to get distance in CM from IR sensor using
// Activty Board built in ADC

// Global Storage for PING & IR Sensor Data:
static uint8_t pingArray[NUMBER_OF_PING_SENSORS] = {0};
static uint8_t irArray[NUMBER_OF_IR_SENSORS] = {0};
#ifdef hasButtons
static uint8_t buttonArray[NUMBER_OF_BUTTON_SENSORS] = {0};
#endif
#ifdef hasLEDs
static uint8_t ledArray[NUMBER_OF_LEDS] = {0};
#endif
#ifdef hasFloorObstacleSensors
static uint8_t floorArray[NUMBER_OF_FLOOR_SENSORS] = {0};
#endif

// For Gyroscope:
// We need this even if there is no Gyro. Just pass 0.0 if it doesn't exist.
// Otherwise I would have to modify the propeller_node.py too.
static float gyroHeading = 0.0;
#ifdef hasGyro
#include "pollGyro.h"
#endif

#include "CalculateChecksum.h"
#include "ReceiveSerialData.h"
#include "SendOverSerial.h"
#include "SerialDataVariables.h"
#include "safetyOverrideCog.h"
#ifdef hasQuickStartBoard
#include "pollPropBoard2.h"
#else
#include "pollPingSensors.h"
#endif

static bool propellerBoardInitialized = false;

#include "broadcastOdometryAndRunMotors.h"

void clearTwistRequest(float *CommandedVelocity, float *angularVelocityOffset) {
  *CommandedVelocity = 0.0;
  *angularVelocityOffset = 0.0;
}

int main() {
  // Open terminal here, because we read from it here
  // and write to it in broadcastOdometryAndRunMotors
  // Close simplex serial terminal
  simpleterm_close();
  // Open global Full Duplex serial connection
  term = fdserial_open(31, 30, 0, 115200);

  dprint(term, "%c%cePropeller Activity Board Started%c", START_MARKER,
         SPECIAL_BYTE, END_MARKER);

  uint8_t pcData[MAXIMUM_RECEIVE_DATA_LENGTH]; // Buffer for receiveData to fill
  uint8_t dataTypeReceived = 0;

#ifdef hasQuickStartBoard
  // Start 2nd Propeller Board Communication cog
  cogstart(&pollPropBoard2, NULL, prop2stack, sizeof prop2stack);
#else
  // Start the local sensor polling cog
  cogstart(&pollPingSensors, NULL, pstack, sizeof pstack);
#endif
  pause(cogStartupDelay);

#ifdef hasGyro
  // Initialize Gyro in the main program
  bus = i2c_newbus(GYRO_SCL_PIN, GYRO_SDA_PIN,
                   0); // New I2C bus SCL = Pin 1, SDA = Pin 0
  int n;
  n = i2c_out(bus, i2cAddr, ctrl3, 1, &cfg3, 1);
  n += i2c_out(bus, i2cAddr, ctrl4, 1, &cfg4, 1);
  n += i2c_out(bus, i2cAddr, ctrl1, 1, &cfg1, 1);
  // Make sure Gyro initialized and stall if it did not.
  if (n != 9) {
    dprint(term, "%c%ceGyro Error: Bytes should be 9, but was %d%c",
           START_MARKER, SPECIAL_BYTE, n, END_MARKER);
  }
  // Start Gyro polling in another cog
  cogstart(&pollGyro, NULL, gyrostack, sizeof gyrostack);
  pause(cogStartupDelay);
#endif

  // Start safetyOverride cog
  cogstart(&safetyOverride, NULL, safetyOverrideStack,
           sizeof safetyOverrideStack);
  pause(cogStartupDelay);

  // Start the Odometry broadcast cog
  cogstart(&broadcastOdometryAndRunMotors, NULL, fstack, sizeof fstack);
  pause(cogStartupDelay);

  bool wasEscaping = false;

  // To hold received commands
  float CommandedVelocity = 0.0;
  float newCommandedVelocity;
  float CommandedAngularVelocity;
  float angularVelocityOffset = 0.0;
  float expectedLeftSpeed;
  float expectedRightSpeed;
  int newInputLeftSpeed = 0;
  int newInputRightSpeed = 0;

  int ROStimeout = ROS_TIMEOUT * 100;
  int timeoutCounter = 0;

  // When these two are true, then we are "initialized"
  bool initDataReceived = false;
  bool settingsDataReceived = false;

  dprint(term, "%c%ceMain Loop Starting%c", START_MARKER, SPECIAL_BYTE,
         END_MARKER);
  while (true) {
    dataTypeReceived = receiveData(term, pcData, MAXIMUM_RECEIVE_DATA_LENGTH);

    if (dataTypeReceived > 0) {
      // If new data exists, fill the appropriate struct with it
      // If we need to test if the data has changed or not,
      // here or elsewhere, use the checksum

      // Pick the correct struct/union/array to fill based on the data type
      uint8_t *arrayToFill;
      uint8_t arrayLength;
      switch (dataTypeReceived) {
      case testDataCharacter:
        arrayToFill = testData.inputArray;
        arrayLength = testDataLength;
        break;
      case initDataCharacter:
        arrayToFill = initData.inputArray;
        arrayLength = initDataLength;
        break;
      case settingsDataCharacter:
        arrayToFill = settingsData.inputArray;
        arrayLength = settingsDataLength;
        break;
      case moveDataCharacter:
        arrayToFill = moveData.inputArray;
        arrayLength = moveDataLength;
        break;
      // TODO: Test robot without LEDs and/or buttons
      case ledDataCharacter:
        arrayToFill = ledData.inputArray;
        arrayLength = ledDataLength;
        break;
      case positionUpdateDataCharacter:
        arrayToFill = positionUpdateData.inputArray;
        arrayLength = positionUpdateDataLength;
        break;
      case abdOverrideDataCharacter:
        arrayToFill = abdOverrideData.inputArray;
        arrayLength = abdOverrideDataLength;
        break;
      }

      // Fill the struct via the array in the union
      for (uint8_t n = 0; n < arrayLength; n++) {
        arrayToFill[n] = pcData[n];
      }

      if (dataTypeReceived == testDataCharacter) {
        // Respond directly to TEST data
        // Manipulate data a little for the test to prove that we can and that
        // our checksum calc is happening
        testData.dataStruct.testByte++;
        testData.dataStruct.testFloat += 1.02;

        // NOTE: Use this as a send data example
        // -------------------------------------
        // Add/update checksum in struct for outgoing data
        testData.dataStruct.dataCheckSum =
            calculateChecksum(testData.inputArray, 0, testDataLength - 1);
        // Send data
        sendOverSerial(testData.inputArray, testDataLength, testDataCharacter);
        // -------------------------------------

        // Clear test data struct to ensure testing never shows old data
        testData.dataStruct.testUnsignedLong = 0;
        testData.dataStruct.testIntOne = 0;
        testData.dataStruct.testIntTwo = 0;
        testData.dataStruct.testIntThree = 0;
        testData.dataStruct.testByte = 0;
        testData.dataStruct.testCharacter = '0';
        testData.dataStruct.testFloat = 0.0;
        testData.dataStruct.dataCheckSum = 0;
      } else if (!initDataReceived && dataTypeReceived == initDataCharacter) {
        // Fill globals with initial data from init packet
        X = initData.dataStruct.X;
        Y = initData.dataStruct.Y;
        Heading = initData.dataStruct.Heading;
        gyroHeading = initData.dataStruct.Heading;
        // One of two parts to tell other cogs it is OK to start
        initDataReceived = true;
      } else if (dataTypeReceived == settingsDataCharacter) {
        // Fill globals with data new data from settings packet
        // These are settings that can change
        trackWidth = settingsData.dataStruct.trackWidth;
        distancePerCount = settingsData.dataStruct.distancePerCount;
        wheelSymmetryError = settingsData.dataStruct.wheelSymmetryError;
        ignoreProximity = settingsData.dataStruct.ignoreProximity;
        ignoreIRSensors = settingsData.dataStruct.ignoreIRSensors;
        ignoreFloorSensors = settingsData.dataStruct.ignoreFloorSensors;
        ignoreCliffSensors = settingsData.dataStruct.ignoreCliffSensors;
        pluggedIn = settingsData.dataStruct.pluggedIn;

        // Confirm to ROS that we got and updated the config data
        // ROS will keep sending this data until we do this.
        configData.dataStruct.trackWidth = trackWidth;
        configData.dataStruct.distancePerCount = distancePerCount;
        configData.dataStruct.wheelSymmetryError = wheelSymmetryError;
        configData.dataStruct.ignoreProximity = ignoreProximity;
        configData.dataStruct.ignoreCliffSensors = ignoreCliffSensors;
        configData.dataStruct.ignoreIRSensors = ignoreIRSensors;
        configData.dataStruct.ignoreFloorSensors = ignoreFloorSensors;
        configData.dataStruct.pluggedIn = pluggedIn;

        // Add/update checksum in struct for outgoing data
        configData.dataStruct.dataCheckSum =
            calculateChecksum(configData.inputArray, 0, configDataLength - 1);
        // Send data
        sendOverSerial(configData.inputArray, configDataLength,
                       configDataCharacter);

        // Two of two parts to tell other cogs it is OK to start
        settingsDataReceived = true;
      } else if (dataTypeReceived == moveDataCharacter) {
        CommandedVelocity = moveData.dataStruct.CommandedVelocity;
        CommandedAngularVelocity = moveData.dataStruct.CommandedAngularVelocity;
        angularVelocityOffset = CommandedAngularVelocity * (trackWidth * 0.5);
        timeoutCounter = 0;
#ifdef hasLEDs
      } else if (dataTypeReceived == ledDataCharacter) {
        ledArray[ledData.dataStruct.ledNumber] = ledData.dataStruct.ledState;
#endif
      } else if (dataTypeReceived == positionUpdateDataCharacter) {
        X = positionUpdateData.dataStruct.X;
        Y = positionUpdateData.dataStruct.Y;
        Heading = positionUpdateData.dataStruct.Heading;
      } else if (dataTypeReceived == abdOverrideDataCharacter) {
        abd_speedLimit = abdOverrideData.dataStruct.abd_speedLimit;
        abdR_speedLimit = abdOverrideData.dataStruct.abdR_speedLimit;
      }
    }

    if (!propellerBoardInitialized) {
      if (initDataReceived && settingsDataReceived) {
        propellerBoardInitialized = true;
        pause(1);
      } else {
        // Send signal that we are "ready"

        // Increment loop counter.
        // This doesn't really accomplish anything, but it is helpful
        // in seeing that things are moving,
        // and provides a basic test of the serial interface.
        readyData.dataStruct.loopCount++;

        // Store sensorDataLength in readyData for relay to PC
        // so that it can decode the odometry data
        readyData.dataStruct.sensorDataCount = sensorDataLength;
        readyData.dataStruct.pingCount = NUMBER_OF_PING_SENSORS;
        readyData.dataStruct.irCount = NUMBER_OF_IR_SENSORS;
        readyData.dataStruct.floorCount = NUMBER_OF_FLOOR_SENSORS;
        readyData.dataStruct.buttonCount = NUMBER_OF_BUTTON_SENSORS;
        readyData.dataStruct.ledCount = NUMBER_OF_LEDS;
        readyData.dataStruct.version = PROPELLER_CODE_VERSION_NUMBER;

        // Add/update checksum in struct for outgoing data
        readyData.dataStruct.dataCheckSum =
            calculateChecksum(readyData.inputArray, 0, readyDataLength - 1);
        // Send data
        sendOverSerial(readyData.inputArray, readyDataLength,
                       readyDataCharacter);
        pause(500); // Loop more slowly if not initialized
      }
    } else {
      // Calculate the new speed and set it

      // Stop if we have not received in input in too long.
      // ROS works by continually sending move commands
      // Without this it is easy to get a runaway robot,
      // or a drifting robot that was left with a final very minute command
      // instead of 0
      if (timeoutCounter > ROStimeout) {
        clearTwistRequest(&CommandedVelocity, &angularVelocityOffset);
        timeoutCounter = 0;
      } else {
        timeoutCounter++;
      }

      if (Escaping) {
        newInputLeftSpeed = escapeLeftSpeed;
        newInputRightSpeed = escapeRightSpeed;
        clearTwistRequest(&CommandedVelocity, &angularVelocityOffset);
        wasEscaping = true;
      } else if (wasEscaping) {
        // Halt robot before continuing normally if we were escaping before now.
        newInputLeftSpeed = 0;
        newInputRightSpeed = 0;
        clearTwistRequest(&CommandedVelocity, &angularVelocityOffset);
        wasEscaping = false;
      } else if (CommandedVelocity >= 0 && (cliff || floorO)) {
        // Cliffs and cats are no joke!
        newInputLeftSpeed = 0;
        newInputRightSpeed = 0;
        clearTwistRequest(&CommandedVelocity, &angularVelocityOffset);
      } else if ((CommandedVelocity > 0 && safeToProceed) ||
                 (CommandedVelocity < 0 && safeToRecede) ||
                 CommandedVelocity == 0) {
        /* Prevent saturation at max wheel speed when a compound command
           is sent.
           Without this, if your max speed is 50, and ROS asks us to set
           one wheel at 50 and the other at 100, we will end up with both
           at 50 changing a turn into a straight line!

           Remember that max speed is variable based on parameters within
           this code, such as proximity to walls, etc.
           */

        // Use forward speed limit for rotate in place.
        if (CommandedVelocity > 0 &&
            (abd_speedLimit * distancePerCount) - fabs(angularVelocityOffset) <
                CommandedVelocity) {
          newCommandedVelocity =
              (abd_speedLimit * distancePerCount) - fabs(angularVelocityOffset);
          // Use abdR_speedLimit for reverse movement.
        } else if (CommandedVelocity < 0 &&
                   -((abdR_speedLimit * distancePerCount) -
                     fabs(angularVelocityOffset)) >
                       CommandedVelocity) { // In theory ROS never requests a
                                            // negative angular velocity, only
                                            // teleop
          newCommandedVelocity = -((abdR_speedLimit * distancePerCount) -
                                   fabs(angularVelocityOffset));
        } else {
          // Not doing this on in place rotations (Velocity = 0)
          // Or if requested speed does not exceed maximum.
          newCommandedVelocity = CommandedVelocity;
        }

        expectedLeftSpeed = newCommandedVelocity - angularVelocityOffset;
        expectedRightSpeed = newCommandedVelocity + angularVelocityOffset;

        expectedLeftSpeed = expectedLeftSpeed / distancePerCount;
        expectedRightSpeed =
            expectedRightSpeed / distancePerCount * wheelSymmetryError;

        newInputLeftSpeed = (int)expectedLeftSpeed;
        newInputRightSpeed = (int)expectedRightSpeed;

      } else {
        // Not safe to proceed in the requested direction, but also not
        // escaping, so just be still until somebody tells us to "back out" of
        // the situation.
        newInputLeftSpeed = 0;
        newInputRightSpeed = 0;
        clearTwistRequest(&CommandedVelocity, &angularVelocityOffset);
      }

      // DHB10 controller only works in even numbers, so let's make life easy on
      // it and ourselves if we are doing any comparisons.
      if (newInputLeftSpeed != 0 && newInputLeftSpeed % 2) {
        newInputLeftSpeed >= 0 ? newInputLeftSpeed++ : newInputLeftSpeed--;
      }
      if (newInputRightSpeed != 0 && newInputRightSpeed % 2) {
        newInputRightSpeed >= 0 ? newInputRightSpeed++ : newInputRightSpeed--;
      }

      broadcastSpeedLeft = newInputLeftSpeed;
      broadcastSpeedRight = newInputRightSpeed;

      // Avoid locking up the Propeller by looping too fast
      pause(1);
    }
  }
}

#pragma clang diagnostic pop