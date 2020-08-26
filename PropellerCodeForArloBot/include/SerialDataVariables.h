// =============================================== //
// * Serial Data Data Types *
// These are the data structs used for
// incoming and outgoing serial data packets.
//
// If you ADD or REMOVE a data type, you must also
// update the list of data types in
// ReceiveSerialData.h under "Set dataLength based on data type"
// ROSInterfaceForArlobot.c under "Pick the correct struct..."
// =============================================== //

// =============================================== //
// This is a companion file to
// arlobot_ros/scripts/PropellerSerialDataPacketTypes.py
// where the data types are defined for the ROS side
// Python code.
// =============================================== //

// =============================================== //
// * Test Data *
// Data used to test serial communications with Propeller
// Test Data is the ONLY packet that is both
// OUTGOING and INCOMING
// =============================================== //

#define testDataCharacter 't'

typedef struct testStruct_st {
  uint32_t testUnsignedLong; //  4 bytes
  int16_t testIntOne;        //  2
  int16_t testIntTwo;        //  2
  int16_t testIntThree;      //  2
  uint8_t testByte;          //  1
  uint8_t testCharacter;     //  1
  float testFloat;           //  4 - See note below in float
                             //      TL;DR: "double" is "float" on Prop
  uint8_t dataCheckSum;      //  1 - Remember to include the checksum
} testStruct_t;              // 17 Total length

#define testDataLength 17

// WARNING: In SimpleIDE there is a box that says "32bit Double" FLOAT is 32
// bit! So what this means is there is no double, double === float SO on the
// Python side use 'f' NOT 'd'!! Or you get a mismatch between pack and the
// struct!

/*
NOTE: I'm trying to switch to 100% C99 integer descriptions, so it is clear to
me how big each number is, and how much memory each is using.
https://arduino.stackexchange.com/questions/30749/int-vs-uint8-t-vs-uint16-t
And a reminder of the types:
https://en.wikipedia.org/wiki/C_data_types
And how Python pack() will handle them:
https://docs.python.org/2/library/struct.html#format-characters
*/

// NOTE: Because the SimpleIDE compiler is set to use 32 bit "doubles" I will
// use only float in my code

// and the struct is overlaid on an array to make it easy to receive data from
// the PC the received data is copied byte by byte into inputArray
//   and can then be used as, e.g. moveData.testUnsignedLong
typedef union {
  testStruct_t dataStruct;
  uint8_t inputArray[testDataLength];
} testUnion;

// this creates a working instance of the Union
// elements in it are referred to as, e.g. dataStruct.moveData.testUnsignedLong
testUnion testData;

// OUTGOING

// =============================================== //
// * Ready Data *
// This is the data format sent to say we are ready,
// but not initialized yet.
// =============================================== //

#define readyDataCharacter 'r'

typedef struct readyStruct_st {
  uint8_t loopCount;       // 1
  uint8_t sensorDataCount; // 1 - NOTE Do not name it the same as the define
                           // sensorDataLength
  uint8_t pingCount;       // 1
  uint8_t irCount;         // 1
  uint8_t floorCount;      // 1
  uint8_t buttonCount;     // 1
  uint8_t ledCount;        // 1
  uint8_t version;         // 1
  uint8_t dataCheckSum;    // 1 - Remember to include the checksum
} readyStruct_t;           // 9 Total length

#define readyDataLength 9

typedef union {
  readyStruct_t dataStruct;
  uint8_t inputArray[readyDataLength];
} readyUnion;

readyUnion readyData;

// =============================================== //
// * Configuration *
// This is the data format sent to transfer
// configuration back to ROS. These are settings that
// either rarely change or only change in response
// to an update from ROS itself. We return them
// both to let ROS know what value we are using,
// in case it changes, and to confirm to ROS
// that we got the message when it sends us a
// settings update.
// =============================================== //

#define configDataCharacter 'c'

typedef struct configStruct_st {
  float trackWidth;         //  4
  float distancePerCount;   //  4
  float wheelSymmetryError; //  4
  bool ignoreProximity;     //  1
  bool ignoreCliffSensors;  //  1
  bool ignoreIRSensors;     //  1
  bool ignoreFloorSensors;  //  1
  bool pluggedIn;           //  1
  uint8_t dataCheckSum;     //  1 - Remember to include the checksum
} configStruct_t;           // 18 Total length

#define configDataLength 18

typedef union {
  configStruct_t dataStruct;
  uint8_t inputArray[configDataLength];
} configUnion;

configUnion configData;

// =============================================== //
// * Odometry *
// This is the data format sent to transfer
// odometry and other telemetry back to ROS.
// =============================================== //

#define odometryDataCharacter 'o'
#define sensorDataLength                                                       \
  (NUMBER_OF_PING_SENSORS + NUMBER_OF_IR_SENSORS + NUMBER_OF_FLOOR_SENSORS +   \
   NUMBER_OF_BUTTON_SENSORS + NUMBER_OF_LEDS)

typedef struct odometryStruct_st {
  float X;                   //  4
  float Y;                   //  4
  float Heading;             //  4
  float gyroHeading;         //  4
  float V;                   //  4
  float Omega;               //  4
  float leftMotorPower;      //  4
  float rightMotorPower;     //  4
  uint8_t abd_speedLimit;    //  1
  uint8_t abdR_speedLimit;   //  1
  uint8_t minDistanceSensor; //  1
  bool safeToProceed;        //  1
  bool safeToRecede;         //  1
  bool Escaping;             //  1
  bool cliff;                //  1
  bool floorO;               //  1
  uint8_t sensorData[sensorDataLength];
  uint8_t dataCheckSum; //  1 - Remember to include the checksum
} odometryStruct_t;     // 41 Total length

#define odometryDataLength (41 + sensorDataLength)

typedef union {
  odometryStruct_t dataStruct;
  uint8_t inputArray[odometryDataLength];
} odometryUnion;

odometryUnion odometryData;

// INCOMING

// =============================================== //
// * Init Data *
// This is the data needed to initialize the robot
// =============================================== //

#define initDataCharacter 'i'

typedef struct initStruct_st {
  float X;              //  4
  float Y;              //  4
  float Heading;        //  4
  uint8_t dataCheckSum; //  1 - Remember to include the checksum
} initStruct_t;         // 13 Total length

#define initDataLength 13

typedef union {
  initStruct_t dataStruct;
  uint8_t inputArray[initDataLength];
} initUnion;

initUnion initData;

// =============================================== //
// * Settings Data *
// This is the data used to set robot parameters
// that are stored on the PC side both before
// the robot is initialized, and in real time
// as the robot operates.
// =============================================== //

#define settingsDataCharacter 's'

typedef struct settingsStruct_st {
  float trackWidth;         //  4
  float distancePerCount;   //  4
  float wheelSymmetryError; //  4
  bool ignoreProximity;     //  1
  bool ignoreCliffSensors;  //  1
  bool ignoreIRSensors;     //  1
  bool ignoreFloorSensors;  //  1
  bool pluggedIn;           //  1
  uint8_t dataCheckSum;     //  1 - Remember to include the checksum
} settingsStruct_t;         // 18 Total length

#define settingsDataLength 18

typedef union {
  settingsStruct_t dataStruct;
  uint8_t inputArray[settingsDataLength];
} settingsUnion;

settingsUnion settingsData;

// =============================================== //
// * Move Data *
// This is the data used to tell the robot to move
// =============================================== //

#define moveDataCharacter 'm'

typedef struct moveStruct_st {
  float CommandedVelocity;        // 4
  float CommandedAngularVelocity; // 4
  uint8_t dataCheckSum;           // 1 - Remember to include the checksum
} moveStruct_t;                   // 9 Total length

#define moveDataLength 9

typedef union {
  moveStruct_t dataStruct;
  uint8_t inputArray[moveDataLength];
} moveUnion;

moveUnion moveData;

// =============================================== //
// * LED Data *
// This is the data used to alter the state
// of LEDs on buttons if your robot has them
// =============================================== //

#define ledDataCharacter 'l'

typedef struct ledStruct_st {
  uint8_t ledNumber;    // 1
  uint8_t ledState;     // 1
  uint8_t dataCheckSum; // 1 - Remember to include the checksum
} ledStruct_t;          // 3 Total length

#define ledDataLength 3

typedef union {
  ledStruct_t dataStruct;
  uint8_t inputArray[ledDataLength];
} ledUnion;

ledUnion ledData;

// =============================================== //
// * Position Update Data *
// Update the X, Y position and Heading
//
// This is not a "normal" action. The robot creates and tracks its own
// odometry, and reports it, not reads it. The ROS program tracks a "difference"
// between the reported odometry and the position on the map. But if you
// know the robot's position better than it does, for instance, when
// loading a new map, or a board reset while Localization is running, then we may
// want to tell the robot where to start after init is already done.
// =============================================== //

#define positionUpdateDataCharacter 'p'

typedef struct positionUpdateStruct_st {
  float X;                //  4
  float Y;                //  4
  float Heading;          //  4
  uint8_t dataCheckSum;   //  1 - Remember to include the checksum
} positionUpdateStruct_t; // 13 Total length

#define positionUpdateDataLength 13

typedef union {
  positionUpdateStruct_t dataStruct;
  uint8_t inputArray[positionUpdateDataLength];
} positionUpdateUnion;

positionUpdateUnion positionUpdateData;

// =============================================== //
// * ABD Parameter Overrides *
// We don't usually adjust these externally,
// but if we want to, this is how.
// Remember this is adjusted in real time by the
// safty override cog, so setting this is a
// fight against the running code.
// =============================================== //

#define abdOverrideDataCharacter 'a'

typedef struct abdOverrideStruct_st {
  uint8_t abd_speedLimit;  // 1
  uint8_t abdR_speedLimit; // 1
  uint8_t dataCheckSum;    // 1 - Remember to include the checksum
} abdOverrideStruct_t;     // 3 Total length

#define abdOverrideDataLength 3

typedef union {
  abdOverrideStruct_t dataStruct;
  uint8_t inputArray[abdOverrideDataLength];
} abdOverrideUnion;

abdOverrideUnion abdOverrideData;

// This is the data buffer size for incoming packets
// It must be as large as the LARGEST POSSIBLE INCOMING packet
// Currently that is the TEST packet at 17,
// but if any other INCOMING packet gets longer, increase this!
#define MAXIMUM_RECEIVE_DATA_LENGTH 17
