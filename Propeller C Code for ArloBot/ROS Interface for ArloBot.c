/* ATTENTION! ATTENTION! ATTENTION! ATTENTION! ATTENTION! ATTENTION!
You MUST edit the settings in
~/.arlobot/per_robot_settings_for_propeller_c_code.h
based on the physical layout of your robot!
For each QUESTION:
UNCOMMENT '#define' lines for any included items,
COMMENT '#define' lines for anything that is not included.
For each SETTING:
Set the variable as required, noting that usually these are ignored if the preceding QUESTION is commented out.

Example, My robot has a "Thing1", but not a "Thing2"
*/
#define hasThingOne
//#define hasTHingTwo

/* Just like that, comment out the "has" line for things you do not have,
and if you do have the thing, adjust the numbers on the other definition as needed.
By using the #define lines, code for items you do not have is never seen by the compiler and is never even loaded on the Propeller bard, saving memory. */

#include "per_robot_settings_for_propeller_c_code.h"
/* If SimpleIDE build fails because the above file is missing,
open up the "Project Manager", then the "Compiler" tab,
and fix the path to your ~/.arlobot/ folder
under Other Compiler Options
and/or copy the above file from the dotfiles folder
to your ~/.arlobot folder.
You could also just move the files in the dotfiles folder into
the folder with this file, but future "git pull" updates
will erase your changes.*/

/*
This is the code to run on a Parallax Propeller based Activity Board
in order to interface ROS with an ArloBot.

Author: Chris L8 https://github.com/chrisl8
URL: https://github.com/chrisl8/ArloBot

The ROS Node for this code is called propellerbot_node.py
and can be found in the arlobot_bringup package from the above URL.

Special thanks to Dr. Rainer Hessmer. Much of this code is based on his work at
https://code.google.com/p/drh-robotics-ros/

Please also see these sites which helped me tremendously with the formulas:
http://www.seattlerobotics.org/encoder/200610/article3/IMU%20Odometry,%20by%20David%20Anderson.htm
http://webdelcire.com/wordpress/archives/527

And of course the entire point of this is to interface with ROS, so read about everything ROS here:
http://wiki.ros.org/

All code here heavily borrowed from everywhere code can be found! :)
*/

/* SimpleIDE Options
 * Everything here is the default for a new SimpleIDE project except for "Enable Pruning."
 * Project Options:
 * Board Type: ACTIVITYBOARD
 * Compiler Type: C - This is C code
 * Memory Model: CMM Main RAM Compact - My code does not fit into LMM, and there is no LMM version of arlodrive.h
 * Optimization: -Os Size - When I change this to "Speed" I get strange behavior.
 * Compiler:
 * CHECK - 32bit Double
 * CHECK - Enable Pruning - This does not make much difference, but a little.
 * Other Compiler Options: -std=c99 - this is part of the default SimpleIDE New project
 * Linker:
 * CHECK - Math lib - required for floating point math!
 * nothing else checked or added under the Linker tab.
 */

#include "simpletools.h"
#include "fdserial.h"

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
// Include adcDCpropab http://learn.parallax.com/propeller-c-simple-circuits/measure-volts
#include "adcDCpropab.h"
#endif

/*
http://forums.parallax.com/showthread.php/154274-The-quot-Artist-quot-robot?p=1277133&viewfull=1#post1277133
"The most impressive is that we can use the same code as the ActivityBot examples, replacing only the library’s name. So everyone who has a big-size Robot, similar to The Artist (like Arlo, Eddie or any other) must only change the library “abdrive.h” with the “arlodrive.h”. So we can take all the advantages form the pre-written code for the ActivityBot!'
http://www.parallax.com/news/2013-12-23/run-activitybot-c-code-arlo
http://forums.parallax.com/showthread.php/152636-Run-ActivityBot-C-Code-in-the-Arlo!?p=1230592&posted=1#post1230592
*/
#include "arlodrive.h"

static int abd_speedLimit = 100; // 100 is default in arlodrive, but we may change it.
static int abdR_speedLimit = 100; // Reverse speed limit to allow robot to reverse fast if it is blocked in front and visa versa

fdserial *term;

// Robot description: We will get this from ROS so that it is easier to tweak between runs without reloading the Propeller EEPROM.
// http://learn.parallax.com/activitybot/calculating-angles-rotation
static double distancePerCount = 0.0, trackWidth = 0.0;
/* See ~/catkin_ws/src/ArloBot/src/arlobot/arlobot_bringup/param/arlobot.yaml
   to set or change this value
*/

const char delimiter[2] = ","; // Delimiter character for incoming messages from the ROS Python script

// For Odometry
int ticksLeft, ticksRight, ticksLeftOld, ticksRightOld;
static double Heading = 0.0, X = 0.0, Y = 0.0;
static int speedLeft, speedRight, throttleStatus = 0;

void getTicks();

void displayTicks();
//void drive_getSpeedCalc(int *left, int *right); // Built into arlodrive.h, not abdrive.h

void broadcastOdometry(void *par); // Use a cog to broadcast Odometry to ROS continuously
static int fstack[256]; // If things get weird make this number bigger!

//int adc_IR_cm(int); // Function to get distance in CM from IR sensor using Activty Board built in ADC

// Global Storage for PING & IR Sensor Data:
int pingArray[NUMBER_OF_PING_SENSORS] = {0};
int irArray[NUMBER_OF_IR_SENSORS] = {0};
#ifdef hasFloorObstacleSensors
int floorArray[NUMBER_OF_FLOOR_SENSORS] = {0};
#endif

#ifdef hasQuickStartBoard
// For Quickstart Board communication
fdserial *propterm;
void pollPropBoard2(void *par); // Use a cog to fill range variables with ping distances
static int prop2stack[128]; // If things get weird make this number bigger!
#else
// Local Sensor Polling Cog
void pollPingSensors(void *par); // Use a cog to fill range variables with ping distances
static int pstack[128]; // If things get weird make this number bigger!
int mcp3208_IR_cm(int); // Function to get distance in CM from IR sensor using MCP3208
#endif

// For Gyroscope:
#ifdef hasGyro
/*
   NOTE About Gyro module:
   Currently this code makes NO use of the gyro. The Odometry from the ArloBot's wheel encoders is excellent!
   The only thing I do with the gyro is send the data to ROS.
   At some point a ROS node could use that data to detect a serious issue, like the robot being picked up or being stuck.
   As it is though, gmapping, AMCL, etc. work very well off of the Odometry with using the gyro data.
   */
unsigned char i2cAddr = 0x69;       //I2C Gyro address
//L3G4200D register addresses & commads.
//See device datasheet section 7 for more info.
unsigned char devId = 0x0f;        //Device ID
unsigned char ctrl1 = 0x20;        //Control reg1
unsigned char cfg1 = 0b00011111;   //100 hz, 25 cutoff, power up, axes enabled
unsigned char ctrl2 = 0x21;
unsigned char ctrl3 = 0x22;
unsigned char cfg3 = 0b00001000;    //Enable data poling (I2_DRDY)
unsigned char ctrl4 = 0x23;
unsigned char cfg4 = 0b10000000;    //Block until read, big endian
unsigned char status = 0x27;
unsigned char xL = 0x28;            //Reg for x low byte - Next 5 bytes xH, yL, yH, zL, xH
unsigned char reply;                //Single byte reply
char xyz[6];                        //XYZ dat array
int gyroXvel, gyroYvel, gyroZvel;                       //Axis variables
i2c *bus;                           //Declare I2C bus
// Create a cog for polling the Gyro
void pollGyro(void *par); // Use a cog to fill range variables with ping distances
static int gyrostack[128]; // If things get weird make this number bigger!
#endif
// We need this even if there is no Gyro. Just pass 0.0 if it doesn't exist.
// Otherwise I would have to modify the propeller_node.py too.
static double gyroHeading = 0.0;

// For "Safety Override" Cog
static volatile int safeToProceed = 0,
safeToRecede = 0,
cliff = 0,
floorO = 0,
Escaping = 0,
minDistanceSensor = 0,
ignoreProximity = 0,
ignoreCliffSensors = 0,
ignoreFloorSensors = 0,
ignoreIRSensors = 0,
pluggedIn = 0;

void safetyOverride(void *par); // Use a cog to squelch incoming commands and perform safety procedures like halting, backing off, avoiding cliffs, calling for help, etc.
// This can use proximity sensors to detect obstacles (including people) and cliffs
// This can use the gyro to detect tipping
// This can use the gyro to detect significant heading errors due to slipping wheels when an obstacle is encountered or high centered
static int safetyOverrideStack[128]; // If things get weird make this number bigger!

int main() {

    simpleterm_close(); // Close simplex serial terminal
    term = fdserial_open(31, 30, 0, 115200); // Open Full Duplex serial connection

    /* Wait for ROS to give us the robot parameters,
       broadcasting 'i' until it does to tell ROS that we
       are ready */
    int robotInitialized = 0; // Do not compute odometry until we have the trackWidth and distancePerCount

    // For Debugging without ROS:
    // See encoders.yaml for most up to date values
    /*
       trackWidth = 0.403000; // from measurement and then testing
       distancePerCount = 0.006760; // http://forums.parallax.com/showthread.php/154274-The-quot-Artist-quot-robot?p=1271544&viewfull=1#post1271544
       robotInitialized = 1;
       */
    // Comment out above lines for use with ROS

    // For PIRsensor
       int PIRhitCounter = 0;
    int personThreshhold = 15; // Must get more than this number of hits before we call it a person.
    int personDetected = 0;

    // Preinitialized loop
    while (robotInitialized == 0) {
        dprint(term, "i\t%d\n", personDetected); // Request Robot distancePerCount and trackWidth NOTE: Python code cannot deal with a line with no divider characters on it.
        pause(10); // Give ROS time to respond, but not too much or we bump into other stuff that may be coming in from ROS.
        if (fdserial_rxReady(term) != 0) { // Non blocking check for data in the input buffer
          const int bufferLength = 65; // A Buffer long enough to hold the longest line ROS may send.
            char buf[bufferLength];
            int count = 0;
            while (count < bufferLength) {
                buf[count] = fdserial_rxTime(term, 100); // fdserial_rxTime will time out. Otherwise a spurious character on the line will cause us to get stuck forever
                if (buf[count] == '\r' || buf[count] == '\n')
                    break;
                count++;
            }

            if (buf[0] == 'd') {
                char *token;
                token = strtok(buf, delimiter);
                token = strtok(NULL, delimiter);
                char *unconverted;
                trackWidth = strtod(token, &unconverted);
                token = strtok(NULL, delimiter);
                distancePerCount = strtod(token, &unconverted);
                token = strtok(NULL, delimiter);
                ignoreProximity = (int)(strtod(token, &unconverted));
                token = strtok(NULL, delimiter);
                ignoreCliffSensors = (int)(strtod(token, &unconverted));
                token = strtok(NULL, delimiter);
                ignoreIRSensors = (int)(strtod(token, &unconverted));
                token = strtok(NULL, delimiter);
                ignoreFloorSensors = (int)(strtod(token, &unconverted));
                token = strtok(NULL, delimiter);
                pluggedIn = (int)(strtod(token, &unconverted));
                token = strtok(NULL, delimiter);
                // Set initial location from ROS, in case we want to recover our last location!
                X = strtod(token, &unconverted);
                token = strtok(NULL, delimiter);
                Y = strtod(token, &unconverted);
                token = strtok(NULL, delimiter);
                Heading = strtod(token, &unconverted);
                gyroHeading = Heading;
                if (trackWidth > 0.0 && distancePerCount > 0.0)
                    robotInitialized = 1;
            }
        } else {
            #ifdef hasPIR
                for (int i = 0; i < 5; i++) { // 5 x 200ms pause = 1000 between updates
                    int PIRstate = input(PIR_PIN); // Check sensor (1) motion, (0) no motion
                    // Count positive hits and make a call:
                    if (PIRstate == 0) {
                        PIRhitCounter = 0;
                    } else {
                        PIRhitCounter++; // Increment on each positive hit
                    }
                    if (PIRhitCounter > personThreshhold) {
                        personDetected = 1;
                    } else {
                        personDetected = 0;
                    }
                    pause(200); // Pause 1/5 second before repeat
                }
            #else
                pause(1000); // Longer pauses when robot is uninitialized
            #endif
            }
        }

    #ifdef hasQuickStartBoard
    // Start 2nd Propeller Board Communication cog
        cogstart(&pollPropBoard2, NULL, prop2stack, sizeof prop2stack);
    #else
    // Start the local sensor polling cog
        cogstart(&pollPingSensors, NULL, pstack, sizeof pstack);
    #endif

    #ifdef hasGyro
        // Initialize Gyro in the main program
        bus = i2c_newbus(GYRO_SCL_PIN, GYRO_SDA_PIN, 0);        //New I2C bus SCL = Pin 1, SDA = Pin 0
        int n;
        n = i2c_out(bus, i2cAddr, ctrl3, 1, &cfg3, 1);
        n += i2c_out(bus, i2cAddr, ctrl4, 1, &cfg4, 1);
        n += i2c_out(bus, i2cAddr, ctrl1, 1, &cfg1, 1);
        // Make sure Gyro initialized and stall if it did not.
        if (n != 9) {
            print("Bytes should be 9, but was %d,", n);
            while (1); // This should just TELL ROS that there is no gyro available instead of stalling the program,
            // TODO:
            // OR have ROS tell us if we HAVE a gyro and only start this if we think we do.
            // That way the program works with or without a gyro
        }
        // Start Gyro polling in another cog
        cogstart(&pollGyro, NULL, gyrostack, sizeof gyrostack);
    #endif

    // Now initialize the Motors
    // abdrive settings:
    drive_speed(0, 0);                     // Start servos/encoders cog
    drive_setMaxSpeed(abd_speedLimit);
    drive_setRampStep(3);              // Set ramping speed
    // TODO: Do we need to adjust ramping? Perhaps this should be something we can modify on the ROS side and send?

    // Start safetyOverride cog: (AFTER the Motors are initialized!)
    cogstart(&safetyOverride, NULL, safetyOverrideStack, sizeof safetyOverrideStack);

    // Start the Odometry broadcast cog
    cogstart(&broadcastOdometry, NULL, fstack, sizeof fstack);

    // To hold received commands
    double CommandedVelocity = 0.0;
    double CommandedAngularVelocity = 0.0;

    // Declaring variables outside of loop
    // This may or may not improve performance
    // Some of these we want to hold and use later too
    // A Buffer long enough to hold the longest line ROS may send.
    const int bufferLength = 35;
    char buf[bufferLength];
    int count = 0;
    double angularVelocityOffset = 0.0, expectedLeftSpeed = 0.0, expectedRightSpeed = 0.0;

    // Listen for drive commands
    int timeoutCounter = 0;
    while (1) {

        timeoutCounter++;

        if (fdserial_rxReady(term) != 0) { // Non blocking check for data in the input buffer
            count = 0;
            while (count < bufferLength) {
                buf[count] = readChar(term);
                if (buf[count] == '\r' || buf[count] == '\n')
                    break;
                count++;
            }

            if (buf[0] == 's') {
                char *token;
                token = strtok(buf, delimiter);
                token = strtok(NULL, delimiter);
                char *unconverted;
                CommandedVelocity = strtod(token, &unconverted);
                token = strtok(NULL, delimiter);
                CommandedAngularVelocity = strtod(token, &unconverted);
                angularVelocityOffset = CommandedAngularVelocity * (trackWidth * 0.5);
                timeoutCounter = 0;
            } else if (buf[0] == 'd') {
                char *token;
                token = strtok(buf, delimiter);
                token = strtok(NULL, delimiter);
                char *unconverted;
                trackWidth = strtod(token, &unconverted);
                token = strtok(NULL, delimiter);
                distancePerCount = strtod(token, &unconverted);
                token = strtok(NULL, delimiter);
                ignoreProximity = (int)(strtod(token, &unconverted));
                token = strtok(NULL, delimiter);
                ignoreCliffSensors = (int)(strtod(token, &unconverted));
                token = strtok(NULL, delimiter);
                ignoreIRSensors = (int)(strtod(token, &unconverted));
                token = strtok(NULL, delimiter);
                ignoreFloorSensors = (int)(strtod(token, &unconverted));
                token = strtok(NULL, delimiter);
                pluggedIn = (int)(strtod(token, &unconverted));
                #ifdef debugModeOn
                dprint(term, "GOT D! %d %d %d %d %d\n", ignoreProximity, ignoreCliffSensors, ignoreIRSensors, ignoreFloorSensors, pluggedIn); // For Debugging
                #endif
                timeoutCounter = 0;
            }
        }

        if (timeoutCounter > ROStimeout) {
            CommandedVelocity = 0.0;
            CommandedAngularVelocity = 0.0;
            angularVelocityOffset = 0.0;
            #ifdef debugModeOn
            dprint(term, "DEBUG: Stopping Robot due to serial timeout.\n");
            #endif
            timeoutCounter = ROStimeout; // Prevent runaway integer length
        }

        /* Restructuring this so that it updates the drive_speed on EVERY
           round. This way even if there is no updated twist command
           from ROS, we will still account for updates in the speed limit
           from the SaftyOverride cog by recalculating the drive commands
           based on the new speed limit at every loop.*/

        // Prevent saturation at max wheel speed when a compound command is sent.
        /* Without this, if your max speed is 50, and ROS asks us to set one wheel
           at 50 and the other at 100, we will end up with both at 50
           changing a turn into a straight line!
           This is because arlodrive.c just cuts off each wheel at the set max speed
           with no regard for the expected left to right wheel speed ratio.
           Here is the code from arlodrive.c:
           int abd_speedLimit = 100;
           static int encoderFeedback = 1;

           void drive_setMaxSpeed(int maxTicksPerSec) {
           abd_speedLimit = maxTicksPerSec;
           }
           ...
           void set_drive_speed(int left, int right) {
           if(encoderFeedback) {
           if(left > abd_speedLimit) left = abd_speedLimit;
           if(left < -abd_speedLimit) left = -abd_speedLimit;
           if(right > abd_speedLimit) right = abd_speedLimit;
           if(right < -abd_speedLimit) right = -abd_speedLimit;
           }}
           ...

           So clearly we need to "normalize" the speed so that if one number is truncated,
           the other is brought down the same amount, in order to accomplish the same turn
           ratio at a slower speed!
           Especially if the max speed is variable based on parameters within this code,
           such as proximity to walls, etc.
           */

        //dprint(term, "\nd1:%f\n", CommandedVelocity); // For Debugging
        // Not doing this on in place rotations (Velocity = 0)
        if (CommandedVelocity > 0) { // Use forward speed limit for rotate in place.
            if ((abd_speedLimit * distancePerCount) - fabs(angularVelocityOffset) < CommandedVelocity)
                CommandedVelocity = (abd_speedLimit * distancePerCount) - fabs(angularVelocityOffset);
            // Use abdR_speedLimit for reverse movement.
        } else if (CommandedVelocity < 0){ // In theory ROS never requests a negative angular velocity, only teleop
            if (-((abdR_speedLimit * distancePerCount) - fabs(angularVelocityOffset)) > CommandedVelocity)
                CommandedVelocity = -((abdR_speedLimit * distancePerCount) - fabs(angularVelocityOffset));
        }

        expectedLeftSpeed = CommandedVelocity - angularVelocityOffset;
        expectedRightSpeed = CommandedVelocity + angularVelocityOffset;

        expectedLeftSpeed = expectedLeftSpeed / distancePerCount;
        expectedRightSpeed = expectedRightSpeed / distancePerCount;

        //dprint(term, "\nd1:%f\n", CommandedVelocity); // For Debugging
        if (Escaping == 0) { // Don't fight with the Propeller escape code!
            if (CommandedVelocity > 0) {
                if (safeToProceed == 1)
                    drive_speed(expectedLeftSpeed, expectedRightSpeed);
                else
                    drive_speed(0, 0);
            } else if (CommandedVelocity < 0) { // In theory ROS never does this, right? Only teleop
                if (safeToRecede == 1)
                    drive_speed(expectedLeftSpeed, expectedRightSpeed);
                else
                    drive_speed(0, 0);
            } else if (CommandedVelocity == 0) {
                // Rotate in place regardless of nearby obstacles.
                drive_speed(expectedLeftSpeed, expectedRightSpeed);
            }
        }
        // Using drive_rampStep does not appear to work with AMCL. It expects the robot to do what it is told, and
        // this doesn't give later commands Especially the command to slow to 0! The drive is left creeping after AMCL
        // tells it to stop!
        // Worse drive_rampStep causes the robot to behave in an almost drunk manner where it wanders left and right
        // instead of going straight ahead.
        // The repeated "twist" statements provide the required "loop".
        pause(10); // Maximum read frequency. TODO: Is this required? Is it the right length?
    }
}

/* Some of the code below came from Dr. Rainer Hessmer's robot.pde
   The rest was heavily inspired/copied from here:
http://forums.parallax.com/showthread.php/154963-measuring-speed-of-the-ActivityBot?p=1260800&viewfull=1#post1260800
*/
void broadcastOdometry(void *par) {

    int dt = CLKFREQ / 10;
    int t = CNT;

    while (1) {
        if (CNT - t > dt) {
            t += dt;
            getTicks();
            displayTicks();
        }
    }
}

void getTicks(void) {
    ticksLeftOld = ticksLeft;
    ticksRightOld = ticksRight;
    drive_getTicks(&ticksLeft, &ticksRight);
    drive_getSpeedCalc(&speedLeft, &speedRight);
}

void displayTicks(void) {
    int deltaTicksLeft = ticksLeft - ticksLeftOld;
    int deltaTicksRight = ticksRight - ticksRightOld;
    double deltaDistance = 0.5f * (double) (deltaTicksLeft + deltaTicksRight) * distancePerCount;
    double deltaX = deltaDistance * (double) cos(Heading);
    double deltaY = deltaDistance * (double) sin(Heading);
    double RadiansPerCount = distancePerCount / trackWidth;
    double deltaHeading = (double) (deltaTicksRight - deltaTicksLeft) * RadiansPerCount;

    X += deltaX;
    Y += deltaY;
    Heading += deltaHeading;
    // limit heading to -Pi <= heading < Pi
    if (Heading > PI) {
        Heading -= 2.0 * PI;
    } else {
        if (Heading <= -PI) {
            Heading += 2.0 * PI;
        }
    }

    // http://webdelcire.com/wordpress/archives/527
    double V = ((speedRight * distancePerCount) + (speedLeft * distancePerCount)) / 2;
    double Omega = ((speedRight * distancePerCount) - (speedLeft * distancePerCount)) / trackWidth;

    // Odometry for ROS
    /*
       I sending ALL of the proximity data (IR and PING sensors) to ROS
       over the "odometry" line, since it is real time data which is just as important
       as the odometry, and it seems like it would be faster to send and deal with one packet
       per cycle rather than two.

       In the propeller node I will convert this to fake laser data.
       I have two goals here:
       1. I want to be able to visualize in RVIZ what the sensors are reporting. This will help with debugging
       situations where the robot gets stalled in doorways and such due to odd sensor readings from angled
       surfaces near the sides.
       2. I also want to use at least some of this for obstacle avoidance in AMCL.
       Note that I do not think that IR and PING data will be useful for gmapping, although it is possible.
       It is just too granular and non specific. It would be nice to be able to use the PING (UltraSonic) data
       to deal with mirrors and targets below the Kinect/Xtion, but I'm not sure how practical that is.
       */
    #ifdef enableOutput
       dprint(term, "o\t%.3f\t%.3f\t%.3f\t%.3f\t%.3f\t%.3f\t", X, Y, Heading, gyroHeading, V, Omega);
    // Send the PING/IR sensor data as a JSON packet:
       dprint(term, "{");
    for (int i = 0; i < NUMBER_OF_PING_SENSORS; i++) { // Loop through all of the sensors
        if (i > 0)
            dprint(term, ",");
        dprint(term, "\"p%d\":%d", i, pingArray[i]);
    }
    for (int i = 0; i < NUMBER_OF_IR_SENSORS; i++) { // Loop through all of the sensors
        if (irArray[i] > 0) // Don't pass the empty IR entries, as we know there are some.
            dprint(term, ",\"i%d\":%d", i, irArray[i]);
    }
    #ifdef hasFloorObstacleSensors
    for (int i = 0; i < NUMBER_OF_FLOOR_SENSORS; i++) { // Loop through all of the sensors
        dprint(term, ",\"f%d\":%d", i, floorArray[i]);
    }
    #endif
    dprint(term, "}\n");
    #endif

    // Send a regular "status" update to ROS including information that does not need to be refreshed as often as the odometry.
    throttleStatus = throttleStatus + 1;
    if (throttleStatus > 9) {
        // Check Motor Power
        double leftMotorPower = 4.69;
        double rightMotorPower = 4.69;
        #ifdef hasMotorPowerMonitorCircuit
        // The input numbers for adc_init are printed right on the Activity Board.
        adc_init(21, 20, 19, 18);
        leftMotorPower = adc_volts(LEFT_MOTOR_ADC_PIN);
        rightMotorPower = adc_volts(RIGHT_MOTOR_ADC_PIN);
        #endif
        dprint(term, "s\t%d\t%d\t%d\t%d\t%d\t%d\t%.2f\t%.2f\t%d\t%d\n", safeToProceed, safeToRecede, Escaping, abd_speedLimit, abdR_speedLimit, minDistanceSensor, leftMotorPower, rightMotorPower, cliff, floorO);
        throttleStatus = 0;
    }
    #ifdef debugModeOn
    dprint(term, "DEBUG: %d %d %d %d %d\n", ignoreProximity, ignoreCliffSensors, ignoreIRSensors, ignoreFloorSensors, pluggedIn);
    #endif
}

volatile int abd_speedL;
volatile int abd_speedR;

// Built into arlodrive.h, not abdrive.h
/*
   void drive_getSpeedCalc(int *left, int *right) {
 *left = abd_speedL;
 *right = abd_speedR;
 }
 */

// For ADC built into Activity Board
/*
   int adc_IR_cm(int channel) {
// http://www.parallax.com/sites/default/files/downloads/28995-Sharp-IR-Datasheet.pdf
int i, goodSamples = 0;
float j, k = 0.0;
for (i=0;i<IRsamples;i++) {
j = adc_volts(channel);                        // Check A/D 0
if(j > 0.395507813) { // Anything lower is probably beyond the sensor's range
k = k + j;
goodSamples = goodSamples + 1;
}
}
float ADCreading = 0.366210938; // Set default to 0.366210938 (same as 300 from MCP3208), Consider 0.366210938, aka. 88cm Out of Range! (Anything over 80 really)
if(goodSamples > 0) {
ADCreading = k/goodSamples;
}
int adc_cm = 27.86 * pow(ADCreading, -1.15); // https://www.tindie.com/products/upgradeindustries/sharp-10-80cm-infrared-distance-sensor-gp2y0a21yk0f/
return(adc_cm);
}
*/

#ifdef hasQuickStartBoard
void pollPropBoard2(void *par) {
    propterm = fdserial_open(QUICKSTART_RX_PIN, QUICKSTART_TX_PIN, 0, 115200);
    pause(100); // Give the serial connection time to come up. Perhaps this is not required?
    const int bufferLength = 10; // Longer than longest possible received line
    char buf[bufferLength];
    int count = 0, pingSensorNumber = 0, irSensorNumber = 0;
    int rateLimit = 10; // This is the incoming rate limiter. Without some limit the entire Propeller will hang.
    while (1) {
        pause(rateLimit);
        // Tell the other end we are alive, so it doesn't just spin pointlessly.
        // It also keeps the sensors quiet when this end is in an idle state.
        dprint(propterm, "i");
        //Do not put any delay here.
        if (fdserial_rxReady(propterm) != 0) {
            //high(26); // LEDs for debugging
            count = 0;
            while (count < bufferLength) {
                buf[count] = readChar(propterm);
                if (buf[count] == '.') // Using . for end of line instead of line break
                    break;
                count++;
            }
            // For Debugging - Test for failing lines
            /* if (buf[0] != 'p' && buf[0] != 'i')
               dprint(term, "%c\n", buf[0]); */

            if (buf[0] == 'p') {
                char *token;
                token = strtok(buf, delimiter);
                token = strtok(NULL, delimiter);
                char *unconverted;
                pingSensorNumber = strtod(token, &unconverted);
                token = strtok(NULL, delimiter);
                if (pingSensorNumber < NUMBER_OF_PING_SENSORS) {
                    pingArray[pingSensorNumber] = strtod(token, &unconverted);
                    // For Debugging:
                    /* dprint(term, "p%d:%3d ", pingSensorNumber, pingArray[pingSensorNumber]);
                       if(pingSensorNumber == 9)
                       dprint(term, "\n"); */
                   }
           } else if (buf[0] == 'i') {
            char *token;
            token = strtok(buf, delimiter);
            token = strtok(NULL, delimiter);
            char *unconverted;
            irSensorNumber = strtod(token, &unconverted);
            token = strtok(NULL, delimiter);
            if (irSensorNumber < NUMBER_OF_IR_SENSORS) {
                irArray[irSensorNumber] = strtod(token, &unconverted);
                    // For Debugging:
                    //dprint(term, "i%d:%3d ", irSensorNumber, irArray[irSensorNumber]);
            }
        }
            //low(26); // LEDs for debugging
    }
    #ifdef hasFloorObstacleSensors
    for (int i = 0; i < NUMBER_OF_FLOOR_SENSORS; i++) {
        floorArray[i] = input(FIRST_FLOOR_SENSOR_PIN + i);
    }
    #endif
}
}
#else
void pollPingSensors(void *par) {
    int ir = 0;
    while (1)                                    // Repeat indefinitely
    {
        for (int i = 0; i < NUMBER_OF_PING_SENSORS; i++) {
            pingArray[i] = ping_cm(FIRST_PING_SENSOR_PIN + i);
            #ifdef hasMCP3208
            // If there is also an IR sensor at this number check it too
            if (i < NUMBER_OF_IR_ON_MCP3208) {
                irArray[i] = mcp3208_IR_cm(i);
            }
            #endif
        }
    }
}

#ifdef hasMCP3208
int mcp3208_IR_cm(int channel) {
    int mcp3208reading = readADC(channel, MCP3208_DINOUT_PIN, MCP3208_CLK_PIN, MCP3208_CS_PIN);
    float mcp3208volts = (float) mcp3208reading * MCP3208_REFERENCE_VOLTAGE / 4096.0;
    int mcp3208cm = 27.86 * pow(mcp3208volts, -1.15); // https://www.tindie.com/products/upgradeindustries/sharp-10-80cm-infrared-distance-sensor-gp2y0a21yk0f/
    return (mcp3208cm);
}
#endif

#endif

#ifdef hasGyro
void pollGyro(void *par) {
    while (1) {
        int ready = 0;                    //Wait until ready
        while (!ready) {
            i2c_in(bus, i2cAddr, status, 1, &ready, 1);
            ready = 1 & (ready >>= 3);
        }

        for (int i = 0; i < 6; i++)        //Get axis bytes
        {
            int regAddr = xL + i;
            i2c_in(bus, i2cAddr, regAddr, 1, &xyz[i], 1);
        }

        //Bytes to int in Degrees Per Second (dps)
        //"Dividing by 114 reduces noise"
        // http://www.parallax.com/sites/default/files/downloads/27911-L3G4200D-Gyroscope-Application-Note.pdf
        // 1 radian/second [rad/s] = 57.2957795130824 degree/second [°/s]
        gyroXvel = (int) (short) ((xyz[1] << 8) + xyz[0]) / 114; // Perhaps use later to detect tipping?
        gyroYvel = (int) (short) ((xyz[3] << 8) + xyz[2]) / 114; // Perhaps use later to detect tipping?
        gyroZvel = (int) (short) ((xyz[5] << 8) + xyz[4]) / 114;

        // If Gyro is running at 100Hz then time between readings should be 10 milliseconds
        double deltaGyroHeading = 0.01 * gyroZvel * 2; // I'm not sure why I have to multiply by two, but I do.
        deltaGyroHeading = deltaGyroHeading * PI / 180.0; // Convert to Radians

        // Discard small variations when motors are not running to eliminate stationary drift
        // Maybe this should be ANY time that speedLeft == speedRight? Then straight lines would stay straight, since
        // ActivityBot appears to travel VERY good straight lines, but they seem to wobble in RVIZ at the moment.
        if (speedLeft == 0 && speedRight == 0) {
            if (deltaGyroHeading < 0.01) { // But accept large changes in case the robot is bumped or moved. Adjust as needed
                deltaGyroHeading = 0.0;
            }
        }

        gyroHeading += deltaGyroHeading;

        // limit heading to -Pi <= heading < Pi
        if (gyroHeading > PI) {
            gyroHeading -= 2.0 * PI;
        } else {
            if (gyroHeading <= -PI) {
                gyroHeading += 2.0 * PI;
            }
        }

        //pause(250); // Pause between reads, or do we need this? Should we read faster? The !ready loop should handle the Gyro's frequency right?
    }
}
#endif

/* TESTS:
   1. Make sure output sensor readings to ROS are near real time.
   2. Make sure "escape" operations are fast and accurate.
   */
   void safetyOverride(void *par) {
    int throttleRamp = 0;
    // Declare all variables up front so they do not have to be created in the loop, only set.
    // This may or may not improve performance.
    int blockedSensor[NUMBER_OF_PING_SENSORS] = {0};
    int i, blockedF = 0, blockedR = 0, foundCliff = 0, floorObstacle = 0, pleaseEscape = 0, minDistance = 255, minRDistance = 255, newSpeedLimit = 100;
    while (1) {
        if (ignoreProximity == 0) {
            // Reset blockedSensor array to all zeros.
            memset(blockedSensor, 0, sizeof(blockedSensor));
            blockedF = 0;
            blockedR = 0;
            pleaseEscape = 0;
            minDistance = 255;

            #ifdef hasCliffSensors
            foundCliff = 0;
            if (ignoreCliffSensors == 0) {
              // Check Cliff Sensors first
              for (i = FIRST_CLIFF_SENSOR; i < FIRST_CLIFF_SENSOR + NUMBER_OF_CLIFF_SENSORS; i++) {
                if (irArray[i] > FLOOR_DISTANCE) {
                  safeToProceed = 0; // Prevent main thread from setting any drive_speed
                  // Stop robot if it is currently moving forward and not escaping
                  // TODO: Can we "chase" the robot off of a cliff, because the rear sensor
                  // would put us into an "Escaping == 1" situation, but we would be moving forward
                  // OVER the cliff instead of back, and thus really need to stop now?!
                  if ((Escaping == 0) && ((speedLeft + speedRight) > 0)) {
                    drive_speed(0, 0);
                  }
                  // Use this to give the "all clear" later if it never gets set
                  blockedF = 1;
                  // Use this to clear the 'cliff' variable later if this never gets set.
                  foundCliff = 1;
                  // Set the global 'cliff' variable so we can see this in ROS.
                  cliff = 1;
                  blockedSensor[2] = 1; // Pretend this is the front sensor, since it needs to back up NOW!
                  pleaseEscape = 1;
                  }
              }
            }
            // Clear the global 'cliff' variable if no cliff was seen.
            if (foundCliff == 0) {
                cliff = 0;
            }
            #endif

            #ifdef hasFloorObstacleSensors
            floorObstacle = 0;
            if (ignoreFloorSensors == 0) {
              for (i = 0; i < NUMBER_OF_FLOOR_SENSORS; i++) {
                if (floorArray[i] == 0) {
                  safeToProceed = 0; // Prevent main thread from setting any drive_speed
                  // Stop robot if it is currently moving forward and not escaping
                  // TODO: Can we "chase" the robot off of a cliff, because the rear sensor
                  // would put us into an "Escaping == 1" situation, but we would be moving forward
                  // OVER the cliff instead of back, and thus really need to stop now?!
                  if ((Escaping == 0) && ((speedLeft + speedRight) > 0)) {
                    drive_speed(0, 0);
                  }
                  // Use this to give the "all clear" later if it never gets set
                  blockedF = 1;
                  // Use this to clear the 'floorO' variable later if this never gets set.
                  floorObstacle = 1;
                  // Set the global 'floorO' variable so we can see this in ROS.
                  floorO = 1;
                  blockedSensor[2] = 1; // Pretend this is the front sensor, since it needs to back up NOW!
                  pleaseEscape = 1;
                  }
              }
            }
            // Clear the global 'floorO' variable if no floor obstacle was seen.
            if (floorObstacle == 0) {
                floorO = 0;
            }
            #endif

            #ifdef hasFrontPingSensors
            // Walk Front PING Sensors to find blocked paths and halt immediately
            for (i = FIRST_FRONT_PING_SENSOR_NUMBER; i < HOW_MANY_FRONT_PING_SENSORS + FIRST_FRONT_PING_SENSOR_NUMBER; i++) {
                // PING Sensors
                if (pingArray[i] < startSlowDownDistance[i]) {
                    if (pingArray[i] <= haltDistance[i] + 1) { // Halt just before.
                        safeToProceed = 0; // Prevent main thread from setting any drive_speed
                        // Stop robot if it is currently moving forward and not escaping
                        if ((Escaping == 0) && ((speedLeft + speedRight) > 0)) {
                            drive_speed(0, 0);
                        }
                        blockedF = 1; // Use this to give the "all clear" later if it never gets set
                        blockedSensor[i] = 1; // Keep track of which sensors are blocked for intelligent escape sequences.
                        // Escape just after, to try make a buffer to avoid back and forthing.
                        if (pingArray[i] < haltDistance[i]) {
                            pleaseEscape = 1;
                        }
                    }
                    // For speed restriction:
                    if (pingArray[i] < minDistance) {
                        minDistance = pingArray[i];
                        minDistanceSensor = i;
                    }
                }
            }
            #endif

            #ifdef hasFrontUpperDeckSensors
            // Walk Upper Deck Sensors
            for (i = FIRST_FRONT_UPPER_SENSOR_NUMBER; i < HOW_MANY_FRONT_UPPER_SENSORS + FIRST_FRONT_UPPER_SENSOR_NUMBER; i++) {
                // PING Sensors
                if (pingArray[i] < startSlowDownDistance[i]) {
                    // Halt just before.
                    if (pingArray[i] <= haltDistance[i] + 1) {
                        // Prevent main thread from setting any drive_speed
                        safeToProceed = 0;
                        // Stop robot if it is currently moving forward and not escaping
                        if ((Escaping == 0) && ((speedLeft + speedRight) > 0)) {
                            drive_speed(0, 0);
                        }
                         // Use this to give the "all clear" later if it never gets set
                        blockedF = 1;
                         // Keep track of which sensors are blocked for intelligent escape sequences.
                        blockedSensor[i] = 1;
                        if (pingArray[i] < haltDistance[i]) {
                            // Escape just after, to try make a buffer to avoid back and forthing.
                            pleaseEscape = 1;
                        }
                    }
                    // For speed restriction:
                    if (pingArray[i] < minDistance) {
                        minDistance = pingArray[i];
                        minDistanceSensor = i;
                    }
                }
            }
            #endif

            #ifdef hasRearPingSensors
            // Walk REAR Sensor Array to find blocked paths and halt immediately
            for (i = FIRST_REAR_PING_SENSOR_NUMBER; i < FIRST_REAR_PING_SENSOR_NUMBER + HOW_MANY_REAR_PING_SENSORS; i++) {
                if (pingArray[i] < startSlowDownDistance[i]) {
                    if (pingArray[i] <= haltDistance[i] + 1) { // Halt just before.
                        safeToRecede = 0; // Prevent main thread from setting any drive_speed
                        // Stop robot if it is currently moving forward and not escaping
                        if ((Escaping == 0) && ((speedLeft + speedRight) < 0)) {
                            drive_speed(0, 0);
                        }
                        blockedR = 1; // Use this to give the "all clear" later if it never gets set
                        blockedSensor[i] = 1; // Keep track of which sensors are blocked for intelligent escape sequences.
                        if (pingArray[i] < haltDistance[i]) // Escape just after, to try make a buffer to avoid back and forthing.
                            pleaseEscape = 1;
                    }
                    // For speed restriction:
                    if (pingArray[i] < minRDistance) {
                        minRDistance = pingArray[i];
                        minDistanceSensor = i;
                    }
                }
            }
            #endif

            #ifdef hasRearUpperDeckSensors
            for (i = FIRST_REAR_UPPER_SENSOR_NUMBER; i < FIRST_REAR_UPPER_SENSOR_NUMBER + HOW_MANY_REAR_UPPER_SENSORS; i++) { // Only use the rear sensors
                // PING Sensors
                if (pingArray[i] < startSlowDownDistance[i]) {
                    if (pingArray[i] <= haltDistance[i] + 1) { // Halt just before.
                        safeToRecede = 0; // Prevent main thread from setting any drive_speed
                        // Stop robot if it is currently moving forward and not escaping
                        if ((Escaping == 0) && ((speedLeft + speedRight) < 0)) {
                            drive_speed(0, 0);
                        }
                        blockedR = 1; // Use this to give the "all clear" later if it never gets set
                        blockedSensor[i] = 1; // Keep track of which sensors are blocked for intelligent escape sequences.
                        if (pingArray[i] < haltDistance[i]) // Escape just after, to try make a buffer to avoid back and forthing.
                            pleaseEscape = 1;
                    }
                    // For speed restriction:
                    if (pingArray[i] < minRDistance) {
                        minRDistance = pingArray[i];
                        minDistanceSensor = i;
                    }
                }
            }
            #endif

            if (ignoreIRSensors == 0) {
              #ifdef hasFrontIRSensors
              // Walk front IR Sensors
              for (i = FIRST_FRONT_IR_SENSOR_NUMBER; i < HOW_MANY_FRONT_IR_SENSORS + FIRST_FRONT_IR_SENSOR_NUMBER; i++) {
                  if (irArray[i] < IRstartSlowDownDistance[i])  {
                      if (irArray[i] <= haltDistance[i] + 1) {
                          // Prevent main thread from setting any drive_speed
                          safeToProceed = 0;
                          // Stop robot if it is currently moving forward and not escaping
                          if ((Escaping == 0) && ((speedLeft + speedRight) > 0)) {
                              drive_speed(0, 0);
                          }
                          // Use this to give the "all clear" later if it never gets set
                          blockedF = 1;
                          // Keep track of which sensors are blocked for intelligent escape sequences.
                          blockedSensor[i] = 1;
                          if (irArray[i] < haltDistance[i]) {
                              pleaseEscape = 1;
                          }
                      }
                      // For speed restriction:
                      if (irArray[i] < minDistance) {
                          minDistance = irArray[i];
                          minDistanceSensor = i;
                      }
                  }
              }
              #endif

              #ifdef hasRearIRSensors
              for (i = FIRST_REAR_IR_SENSOR_NUMBER; i < FIRST_REAR_IR_SENSOR_NUMBER + HOW_MANY_REAR_IR_SENSORS; i++) {
                  #ifdef RENAME_REAR_IR_SENSOR
                  int sensorFakeIndex = RENAME_REAR_IR_SENSOR;
                  #else
                  int sensorFakeIndex = i;
                  #endif
                  if (irArray[i] < IRstartSlowDownDistance[i]) {
                     if (irArray[i] <= haltDistance[sensorFakeIndex] + 1) {
                          safeToRecede = 0; // Prevent main thread from setting any drive_speed
                          // Stop robot if it is currently moving forward and not escaping
                          if ((Escaping == 0) && ((speedLeft + speedRight) < 0)) {
                              drive_speed(0, 0);
                          }
                          blockedR = 1; // Use this to give the "all clear" later if it never gets set
                          blockedSensor[sensorFakeIndex] = 1; // Keep track of which sensors are blocked for intelligent escape sequences.
                          if (irArray[i] < haltDistance[sensorFakeIndex])
                              pleaseEscape = 1;
                      }
                      // For speed restriction:
                      if (irArray[i] < minRDistance) {
                          minRDistance = irArray[i];
                          minDistanceSensor = i;
                      }
                  }
              }
            #endif
            }

            // Reduce Speed Limit when we are close to an obstruction
            /* EXPLANATION minDistance won't be set unless a given sensor is closer than its particular startSlowDownDistance value, so we won't be slowing down if sensor 0 is 40, only if it is under 10 */
            if (minDistance < MAX_DISTANCE) {
                // Set based on percentage of range
                // TODO: Is this a good method?
                newSpeedLimit = (minDistance - haltDistance[minDistanceSensor]) * (100 / (MAX_DISTANCE - haltDistance[minDistanceSensor]));
                // Limit maximum and minimum speed.
                if (newSpeedLimit < MINIMUM_SPEED) {
                    newSpeedLimit = MINIMUM_SPEED;
                } else if (newSpeedLimit > 100) {
                    newSpeedLimit = 100;
                }
                // Ramp and limit affect of random hits
                if (newSpeedLimit > abd_speedLimit) {
                    if (throttleRamp == THROTTLE_STOP) {
                        abd_speedLimit = abd_speedLimit + 1;
                    }
                } else if (newSpeedLimit < abd_speedLimit) {
                    if (throttleRamp == THROTTLE_STOP) {
                        abd_speedLimit = abd_speedLimit - 1;
                    }
                }
            } else {
                // Ramp return to full if all obstacles are clear
                if (abd_speedLimit < 100) {
                    if (throttleRamp == THROTTLE_STOP) // Slow ramping down
                        abd_speedLimit = abd_speedLimit + 1;
                }
            }

            // Same for REVERSE Speed Limit
            if (minRDistance < MAX_DISTANCE) {
                // Set based on percentage of range
                // TODO: Is this a good method?
                newSpeedLimit = (minRDistance - haltDistance[minDistanceSensor]) * (100 / (MAX_DISTANCE - haltDistance[minDistanceSensor]));
                // Limit maximum and minimum speed.
                if (newSpeedLimit < MINIMUM_SPEED) {
                    newSpeedLimit = MINIMUM_SPEED;
                } else if (newSpeedLimit > 100) {
                    newSpeedLimit = 100;
                }
                // Ramp and limit affect of random hits
                if (newSpeedLimit > abdR_speedLimit) {
                    if (throttleRamp == THROTTLE_STOP) {
                        abdR_speedLimit = abdR_speedLimit + 1;
                    }
                } else if (newSpeedLimit < abdR_speedLimit) {
                    if (throttleRamp == THROTTLE_STOP) {
                        abdR_speedLimit = abdR_speedLimit - 1;
                    }
                }
            } else {
                // Ramp return to full if all obstacles are clear
                if (abdR_speedLimit < 100) {
                    if (throttleRamp == THROTTLE_STOP) // Slow ramping down
                        abdR_speedLimit = abdR_speedLimit + 1;
                }
            }

            // Clear forward and backward individually now.
            if (blockedF == 0) {
                safeToProceed = 1;
            }
            if (blockedR == 0) {
                safeToRecede = 1;
            }
            // If NO sensors are blocked, give the all clear!
            if (blockedF == 0 && blockedR == 0) {
                if (Escaping == 1) {// If it WAS escaping before stop it before releasing it
                    drive_speed(0, 0); // return to stopped before giving control back to main thread
                }
                Escaping = 0; // Have fun!
            } else {
                if (pleaseEscape == 1 && pluggedIn == 0) {
                    // If it is plugged in, don't escape!
                    Escaping = 1; // This will stop main thread from driving the motors.
                    /* At this point we are blocked, so it is OK to take over control
                       of the robot (safeToProceed == 0, so the main thread won't do anything),
                       and it is safe to do work ignoring the need to slow down or stop
                       because we know our position pretty well.
                       HOWEVER, you will have to RECHECK distances yourself if you are going to move
                       in this program location.
                       */
                       if (safeToRecede == 1) {
                        // The order here determines priority.
                        #ifdef FRONT_CENTER_SENSOR
                        if (blockedSensor[FRONT_CENTER_SENSOR] == 1) {
                            drive_speed(-MINIMUM_SPEED, -MINIMUM_SPEED);
                        #ifdef FRONT_3D_MOUNTED_SENSOR
                        } else if (blockedSensor[FRONT_3D_MOUNTED_SENSOR] == 1) {
                            drive_speed(-MINIMUM_SPEED, -MINIMUM_SPEED);
                        #endif
                        #ifdef FRONT_UPPER_DECK_CENTER_SENSOR
                        } else if (blockedSensor[FRONT_UPPER_DECK_CENTER_SENSOR] == 1) {
                            drive_speed(-MINIMUM_SPEED, -MINIMUM_SPEED);
                        #endif
                        #ifdef FRONT_NEAR_LEFT_SENSOR
                        } else if (blockedSensor[FRONT_NEAR_LEFT_SENSOR] == 1) {
                            drive_speed(-MINIMUM_SPEED, -(MINIMUM_SPEED * 2)); // Curve out to the right
                        #endif
                        #ifdef FRONT_UPPER_DECK_NEAR_LEFT_SENSOR
                        } else if (blockedSensor[FRONT_UPPER_DECK_NEAR_LEFT_SENSOR] == 1) {
                            drive_speed(-MINIMUM_SPEED, -(MINIMUM_SPEED * 2)); // Curve out to the right
                        #endif
                        #ifdef FRONT_NEAR_RIGHT_SENSOR
                        } else if (blockedSensor[FRONT_NEAR_RIGHT_SENSOR] == 1) {
                            drive_speed(-(MINIMUM_SPEED * 2), -MINIMUM_SPEED); // Curve out to the left
                        #endif
                        #ifdef FRONT_UPPER_DECK_NEAR_RIGHT_SENSOR
                        } else if (blockedSensor[FRONT_UPPER_DECK_NEAR_RIGHT_SENSOR] == 1) {
                            drive_speed(-(MINIMUM_SPEED * 2), -MINIMUM_SPEED); // Curve out to the left
                        #endif
                        #ifdef FRONT_FAR_LEFT_SENSOR
                        } else if (blockedSensor[FRONT_FAR_LEFT_SENSOR] == 1) {
                            drive_speed(0, -MINIMUM_SPEED); // Turn out to the right slowly
                        #endif
                        #ifdef FRONT_FAR_RIGHT_SENSOR
                        } else if (blockedSensor[FRONT_FAR_RIGHT_SENSOR] == 1) {
                            drive_speed(-MINIMUM_SPEED, 0); // Turn out to the left slowly
                        #endif
                        }
                        #endif
                    } else if (safeToProceed == 1) { // Escaping for rear sensors, these will move more generically forward.
                        #ifdef REAR_CENTER_SENSOR
                        if (blockedSensor[REAR_CENTER_SENSOR] == 1) {
                            drive_speed(MINIMUM_SPEED, MINIMUM_SPEED);
                        #ifdef REAR_3D_MOUNTED_SENSOR
                        } else if (blockedSensor[REAR_3D_MOUNTED_SENSOR] == 1) {
                            drive_speed(MINIMUM_SPEED, MINIMUM_SPEED);
                        #endif
                        #ifdef REAR_UPPER_DECK_SENSOR
                        } else if (blockedSensor[REAR_UPPER_DECK_SENSOR] == 1) {
                            drive_speed(MINIMUM_SPEED, MINIMUM_SPEED);
                        #endif
                        #ifdef REAR_NEAR_RIGHT_SENSOR
                        } else if (blockedSensor[REAR_NEAR_RIGHT_SENSOR] == 1) {
                            drive_speed(MINIMUM_SPEED, MINIMUM_SPEED * 2);
                        #endif
                        #ifdef REAR_NEAR_LEFT_SENSOR
                        } else if (blockedSensor[REAR_NEAR_LEFT_SENSOR] == 1) {
                            drive_speed(MINIMUM_SPEED * 2, MINIMUM_SPEED);
                        #endif
                        #ifdef REAR_FAR_RIGHT_SENSOR
                        } else if (blockedSensor[REAR_FAR_RIGHT_SENSOR] == 1) {
                            drive_speed(MINIMUM_SPEED, 0);
                        #endif
                        #ifdef REAR_FAR_LEFT_SENSOR
                        } else if (blockedSensor[REAR_FAR_LEFT_SENSOR] == 1) {
                            drive_speed(0, MINIMUM_SPEED);
                        #endif
                        }
                        #endif
                    } else { // We are trapped!!
                        // Turns out we cannot escape, so turn off "Escaping",
                        // and now drive control should refuse to move forward or back,
                        // due to safeToRecede & safeToProceed both == 1,
                        // but it should be willing to rotate in place,
                        // which is a normal function of both arlobot_explore
                        // and the navigation stack's "clearing" function.
                        Escaping = 0;
                    }
                } else { // This is the "halt" but don't "escape" action for that middle ground.
                    if (Escaping == 1) {// If it WAS Escaping, stop it now.
                        drive_speed(0, 0); // return to stopped before giving control back to main thread
                    }
                    Escaping = 0; // Blocked, but not close enough to Escape yet
                }
            }

            throttleRamp = throttleRamp + 1;
            if(throttleRamp > THROTTLE_STOP)
                throttleRamp = 0;

        } else {
            /* All limits and blocks must be cleared if we are going to ignore
            proximity. Otherwise we get stuck! */
            Escaping = 0;
            safeToProceed = 1;
            safeToRecede = 1;
            abd_speedLimit = 100;
            abdR_speedLimit = 100;
        }
        pause(1); // Just throttles this cog a little.
    }
}
