/* ATTENTION! ATTENTION! ATTENTION! ATTENTION! ATTENTION! ATTENTION!
You MUST edit the following settings based on the physical layout
of your robot!
For each QUESTION:
UNCOMMENT '#define' lines for any included items,
COMMENT '#define' lines for anything that is not included.
For each SETTING:
Set the variable as required, noting that usually these are ignored if the
preceding QUESTION is commented out.

Example, My robot has a "Thing1", but not a "Thing2"
*/
#define hasThingOne
//#define hasTHingTwo

/* Just like that, comment out the "has" line for things you do not have,
and if you do have the thing, adjust the numbers on the other definition as
needed.
By using the #define lines, code for items you do not have is never seen by the
compiler and is never even loaded on the Propeller bard, saving memory. */

/* NOTICE NOTICE NOTICE
Changes to this file will NOT have any affect until you RELOAD the C code onto
the propeller board with SimpleIDE!!!
*/

// PROXIMITY SENSOR TUNING
// Use these to tune the robots responses!
// What is the maximum distance at which sensor readings should be noticed?
#define MAX_DISTANCE 70
// What is the maximum distance at which IR (Infrared) sensor readings should be
// noticed? IR sensors tend to be less reliable, so make this shorter than the
// MAX_DISTANCE, which applies to PING sensors.
#define IR_MAX_DISTANCE 50
// What is the minimum speed that the speed limiter should set the robot to?
#define MINIMUM_SPEED 20
// THROTTLE_STOP determines how quickly the speed limit is changed.
// Make this number bigger to cause the speed to change more slowly.
#define INCREASE_THROTTLE_RATE 50
#define DECREASE_THROTTLE_RATE 5

// QUESTION: Does your robot have a 2nd Propeller "QuickStart" board where the
// PING sensors are connected?
//#define hasQuickStartBoard
// WARNING WARNING WARNING!
// You must ALSO open up "2nd Board Code.c" and make changes there if you are
// using the QuickStart board! Settings for QuickStart Board - IGNORE IF YOU
// DON'T HAVE ONE! SETTING: Which pin on the Activity Board is the RX pin
// connected out to the QuickStart board?
#define QUICKSTART_RX_PIN 5
// SETTING: Which pin on the Activity Board is the TX pin connected in from the
// QuickStart board?
#define QUICKSTART_TX_PIN 6

// Settings for PING and IR (Infrared) sensors connected DIRECTLY to the
// Activity Board: SETTING: Which pin on the Activity Board is the first PING
// sensor connected to? Also used by the 2nd Quickstart board in my code.
#define FIRST_PING_SENSOR_PIN 5

// QUESTION: Are you using an MCP3208 chip to read poll analog sensors from the
// Activity Board?
//#define hasMCP3208
// Settings for MCP3208, if it has one:
// SETTING: How many IR sensors (if any) are connected to your MCP3208?
#define NUMBER_OF_IR_ON_MCP3208 0
#define MCP3208_DINOUT_PIN 3
#define MCP3208_CLK_PIN 4
#define MCP3208_CS_PIN 2
// MCP3208 reference voltage setting. I use 5.0v for the 5.0v IR sensors from
// Parallax
#define MCP3208_REFERENCE_VOLTAGE 5.0

// PROXIMITY SENSORS:
// IMPORTANT! This section is very important!
// SETTINGS: Proximity (PING & IR) Sensor count and location:
#define NUMBER_OF_PING_SENSORS 1
#define NUMBER_OF_IR_SENSORS 0

// QUESTION: Do you have PING sensors on the front of your robot?
#define hasFrontPingSensors
#define FIRST_FRONT_PING_SENSOR_NUMBER 0 // Count from 0
// Your sensors need to poll consecutively for each section!
#define HOW_MANY_FRONT_PING_SENSORS 1

// QUESTION: Do you have PING sensors on the back of your robot?
//#define hasRearPingSensors
#define FIRST_REAR_PING_SENSOR_NUMBER 5 // Count from 0
#define HOW_MANY_REAR_PING_SENSORS 5

// QUESTION: Do you have IR sensors on the front of your robot?
//#define hasFrontIRSensors
#define FIRST_FRONT_IR_SENSOR_NUMBER 0 // Count from 0
#define HOW_MANY_FRONT_IR_SENSORS 5

// QUESTION: Do you have IR sensors on the back of your robot?
//#define hasRearIRSensors
#define FIRST_REAR_IR_SENSOR_NUMBER 7 // Count from 0
#define HOW_MANY_REAR_IR_SENSORS 1

// QUESTION: Do you have PING sensors on the front "upper deck" of your robot?
/* These would be facing straight forward in the middle of decks above
the deck the circle of sensors is on.
I have one on the top deck, and then one on the stand holding the Xtion sensor.
All of these will be assumed to face forward. */
//#define hasFrontUpperDeckSensors
#define FIRST_FRONT_UPPER_SENSOR_NUMBER 10 // Count from 0
#define HOW_MANY_FRONT_UPPER_SENSORS 3

// QUESTION: Do you have PING sensors on the REAR "upper deck" of your robot?
//#define hasRearUpperDeckSensors
#define FIRST_REAR_UPPER_SENSOR_NUMBER 13 // Count from 0
#define HOW_MANY_REAR_UPPER_SENSORS 1

/* Here you set up arrays showing what the halt distance,
   and distance to start slowing down is for each sensor.
   Each sensor needs to have its own response because of their various angles.
   We cannot use the same distance response for each,
   and we may want different max speeds and escape sequences too.
   Remember the order is based on the order that they are plugged
   in to your board.
   WARNING: Be sure the array has the correct number of entries! */
const int haltDistance[NUMBER_OF_PING_SENSORS] = {12};
// const int haltDistance[NUMBER_OF_PING_SENSORS] = {5, 10, 12, 10, 5, 5, 10,
// 12, 10, 5, 10, 12, 10, 12};
const int startSlowDownDistance[NUMBER_OF_PING_SENSORS] = {MAX_DISTANCE};
// const int startSlowDownDistance[NUMBER_OF_PING_SENSORS] = {8, 40,
// MAX_DISTANCE, 40, 8, 8, 40, MAX_DISTANCE, 40, 8, 40, MAX_DISTANCE, 40,
// MAX_DISTANCE};
// Set shorter distances for IR sensors because they are less reliable.
//const int IRstartSlowDownDistance[NUMBER_OF_IR_SENSORS] = {
//    8, 20, IR_MAX_DISTANCE, 20, 8, IR_MAX_DISTANCE, 0, 0};
/* My last two IR sensors are cliff sensors, thus the "0, 0" because slowdown
does not apply to them. */

// QUESTION: Sensor Identities:
// Put the sensor number by each label
// Comment out any sensor you do not have
#define FRONT_CENTER_SENSOR 2
// NOTE: IF you comment out the FRONT_CENTER_SENSOR, then ALL Front sensors are
// ignored for escaping! "Left" refers to the ROBOT's left, assuming he faces
// the way he drives "forward"
//#define FRONT_NEAR_LEFT_SENSOR 1
//#define FRONT_FAR_LEFT_SENSOR 0
//#define FRONT_NEAR_RIGHT_SENSOR 3
//#define FRONT_FAR_RIGHT_SENSOR 4
//#define REAR_FAR_RIGHT_SENSOR 5
//#define REAR_NEAR_RIGHT_SENSOR 6
//#define REAR_CENTER_SENSOR 7
// NOTE: IF you comment out the REAR_CENTER_SENSOR, then ALL Rear sensors are
// ignored for escaping!
//#define REAR_NEAR_LEFT_SENSOR 8
//#define REAR_FAR_LEFT_SENSOR 9
//#define FRONT_UPPER_DECK_NEAR_LEFT_SENSOR 10
//#define FRONT_UPPER_DECK_CENTER_SENSOR 11
//#define FRONT_UPPER_DECK_NEAR_RIGHT_SENSOR 12
//#define FRONT_3D_MOUNTED_SENSOR 11
//#define REAR_UPPER_DECK_SENSOR 13
//#define REAR_3D_MOUNTED_SENSOR 13

// QUESTION: Do you want to "rename" the rear IR sensor(s)?
/* If this is set, all "rear" IR sensors will be labeled with this number
when checked. I use this because while I have 5 rear PING sensors,
I only have 1 rear IR sensor. So the escape options for the one rear
IR sensor should match the rear center PING sensor.
If you don't want to do this, just comment this setting out:
*/
//#define RENAME_REAR_IR_SENSOR REAR_CENTER_SENSOR

// QUESTION: Does your robot have a PIR (Passive Infrared) sensor connected to
// the Activity Board?
/* NOTE: I got this PIR sensor for free, and it is fun,
   but it doesn't do much for you when the robot is moving
   and sending out IR signals with the Kinect/Xtion
   and with the IR distance sensors. :)
   It is cool though to have it watch for people and alert
   ROS when someone is around. Then ROS can initialize the ArloBot,
   start up the motors with a USB Relay Board
   and start chasing them . . . or something.
   */
//#define hasPIR
// SETTING: IF you have a PIR sensor, which pin is it connected to?
#define PIR_PIN 11

// GYROSCOPE:
// QUESTION: Does your robot have a Gyro module installed on the Activity Board?
//#define hasGyro
// Settings for Gyroscope, if it has one:
// SETTING: Activity Board pin that the SCL pin from the Gyro is connected to:
#define GYRO_SCL_PIN 1
// SETTING: Activity Board pin that the SDA pin from the Gyro is connected to:
#define GYRO_SDA_PIN 0

// VOLTAGE MONITORING:
// QUESTION: Do you have a voltage divider connected to the Activity Board's
// built in ADC to monitor voltage at the left and right motors?
//#define hasMotorPowerMonitorCircuit
// Settings for Motor Power Monitor Circuit, if it has one:
// If these get flipped just flip the wires, or the numbers.
#define LEFT_MOTOR_ADC_PIN 0
#define RIGHT_MOTOR_ADC_PIN 1
// Value for reported left and right motor ADC voltage when
// hasMotorPowerMonitorCircuit is not defined. A value less than 1 will prevent
// the robot from moving under ROS control in propellerbot_node.py unless that
// file is modified. In propellerbot_node.py, the ratio 15/4.69 is used to
// convert the right motor ADC voltage to measured and reported robot battery
// voltage.
#define DEFAULT_MOTOR_ADC_VOLTAGE (12.0 * (4.69 / 15.0))

// CLIFF SENSORS:
// QUESTION: Do you have IR "cliff" sensors mounted to the front of the robot?
//#define hasCliffSensors
// SETTING: Which IR sensor number (counting from 0) is the first cliff sensor?
#define FIRST_CLIFF_SENSOR 5
// SETTING: How many cliff sensors are there?
#define NUMBER_OF_CLIFF_SENSORS 2
// SETTING: What is the maximum distance before the robot should consider it to
// be a cliff? at 30 it was stalling when crossing half inch bumps.
#define FLOOR_DISTANCE 40

// FLOOR OBSTACLE SENSORS:
// QUESTION: Do you have digital infrared sensors on the front of the robot?
//#define hasFloorObstacleSensors
#define FIRST_FLOOR_SENSOR_PIN 7
#define NUMBER_OF_FLOOR_SENSORS 4

// Quickstart Board Settings
// I have a SECOND Propeller chip on a Quickstart board that polls the PING and
// IR sensors, so this is settings for that. Which pin on the QuickStart Board
// is the RX pin connected out to the Activity board?
#define ACTIVITYBOARD_RX_PIN 0
// Which pin on the QuickStart Board is the TX pin connected in from the
// Activity board?
#define ACTIVITYBOARD_TX_PIN 1

// Buttons:
// QUESTION: Do you have button sensors on the robot?
// #define hasButtons
#define NUMBER_OF_BUTTON_SENSORS 0

// I have LEDs hooked up to my quickstart board, well, LEDs on the buttons
// Be sure to add/update these in he 2nd_board.h file too!
// #define hasLEDs
#define NUMBER_OF_LEDS 0

/* END OF QUESTION SECTION!
That is all, you are ready to attempt to build this code and load it
onto your Propeller Activity board!
*/

// Maximum speed in ticks per second. Even if ROS asks us to go faster, we will
// not. This is used to set the abd_speedLimit and abdR_speedLimit
// Two TPS faster than what we are telling ROS is the maximum speed:
#define MAXIMUM_SPEED 220
// Use MotorResponseTesting.c if you want to test/adjust this,
// And be sure to edit the ROS config files with the maximum linear (m/s) and
// angular (rad/s) velocity

// Acceleration rate for DBH-10
// The range is 0 to 32767 with a default of 512
// according to
// https://www.parallax.com/sites/default/files/downloads/28231-DHB-10-Arlo-Firmware-Guide-v1.0.pdf
// ROS DWA doesn't like it when the robot throttles acceleration. So we are
// ramping it up a bit from the default.
#define DHB10_ACC 1024
// Use MotorResponseTesting.c if you want to test/adjust this,
// And be sure to edit the ROS config files with the maximum linear (m/s^2) and
// angular (rad/s^2) acceleration Calculator:
// http://www.smartconversion.com/unit_calculation/Acceleration_calculator.aspx

/* Connect Encoder Pins directly to the Propeller board,
instead of using the DHB10.
The DBH10 reads the encoder pins to give the wheel speed,
however, some user have found the DBH10 to be unreliable, so they hook
the encoder pins to the Propeller directly.
*/
// #define encoderConnectedToPropeller
// Define the encoders pins if connected directly to Propeller
#define LEFT_A 3
#define LEFT_B 2
#define RIGHT_A 1
#define RIGHT_B 0

/* Pause between cog starts
This just sets a delay between each cog start,
mostly so that startup messages don't collide,
but it may help in some other way too. */
#define cogStartupDelay 100

/* These settings affect the speed at which the "loop" on the robot runs.
If it runs too fast we overwhelm serial connections and crash things or get
garbage. Running too slow will reduce the responsiveness of the robot. Remember
though that going faster and "hanging" isn't more responsive.
*/
#define dhb10OverloadPause                                                     \
  2 // Pause before each read/write to DHB10 to avoid overloading it.
// 1 seems to work fine. 2 seems perfectly safe without any ill affects. I
// suggest filing a github issue before increasing this.

/* Timeout setting. After this many seconds the robot will stop if it
has not received a twist command from ROS.

It is often desirable to set this longer when doing direct to serial testing.

However, ROS typically updates the robot frequently, so this should work with
ROS.
*/
#define ROS_TIMEOUT 5
