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
#include "mcp3208.h" // MCP3208 8 Chanel ADC
// Add adcDCpropab.h if you want to use the ADC built into the Activity Board
//#include "adcDCpropab.h"                      // Include adcDCpropab http://learn.parallax.com/propeller-c-simple-circuits/measure-volts
#include "fdserial.h"
#include "ping.h"                             // Include ping header
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
static double distancePerCount = 0.0; // See encoders.yaml to set or change this value
static double trackWidth = 0.0; // See encoders.yaml to set or change this value

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

// for Proximity (PING & IR) Sensors
int pingArray[6];
int irArray[6];
const int numberOfIRonMC3208 = 6; // Number of IR Sensors on the MCP3208 ADC to read
const int numberOfPINGsensors = 6; // Number of PING sensors, we assume the are consecutive
const int firtPINGsensorPIN = 5; // Which pin the first PING sensor is on
void pollPingSensors(void *par); // Use a cog to fill range variables with ping distances
static int pstack[128]; // If things get weird make this number bigger!
int mcp3208_IR_cm(int); // Function to get distance in CM from IR sensor using MCP3208
//int adc_IR_cm(int); // Function to get distance in CM from IR sensor using Activty Board built in ADC

// Gyro globals:
const int hasGyro = 1; // Set this to 0 if you do not have a gyro on your ArloBot's Activity Board.
/*
Currently this code makes NO use of the gyro. The Odometry from the ArloBot's wheel encoders is excellent!
The only thing I do with the gyro is send the data to ROS.
At some point a ROS node could use that data to detect a serious issue, like the robot being picked up or being stuck.
As it is though, gmapping, AMCL, etc. work very well off of the Odometry with using the gyro data.
*/
// For Gyroscope - Declare everything globally
const int gyroSCLpin = 1; // Activity Board pin that the SCL pin from the Gyro is connected to.
const int gyroSDApin = 0;  // Activity Board pin that the SDA pin from the Gyro is connected to.
unsigned char i2cAddr = 0x69;       //I2C Gyro address
//L3G4200D register addresses & commads.
//See device datasheet section 7 for more info.
unsigned char devId  = 0x0f;        //Device ID
unsigned char ctrl1  = 0x20;        //Control reg1
unsigned char cfg1   = 0b00011111;   //100 hz, 25 cutoff, power up, axes enabled
unsigned char ctrl2  = 0x21;
unsigned char ctrl3  = 0x22;
unsigned char cfg3   = 0b00001000;    //Enable data poling (I2_DRDY)
unsigned char ctrl4  = 0x23;
unsigned char cfg4   = 0b10000000;    //Block until read, big endian
unsigned char status = 0x27;
unsigned char xL     = 0x28;            //Reg for x low byte - Next 5 bytes xH, yL, yH, zL, xH
unsigned char reply;                //Single byte reply
char xyz[6];                        //XYZ dat array
int gyroXvel, gyroYvel, gyroZvel;                       //Axis variables
static double gyroHeading = 0.0;
i2c *bus;                           //Declare I2C bus
// Create a cog for polling the Gyro
void pollGyro(void *par); // Use a cog to fill range variables with ping distances
static int gyrostack[128]; // If things get weird make this number bigger!

// For "Safety Override" Cog
static int safeToProceed = 0, safeToRecede = 0, Escaping = 0;
void safetyOverride(void *par); // Use a cog to squelch incoming commands and perform safety procedures like halting, backing off, avoiding cliffs, calling for help, etc.
// This can use proximity sensors to detect obstacles (including people) and cliffs
// This can use the gyro to detect tipping
// This can use the gyro to detect significant heading errors due to slipping wheels when an obstacle is encountered or high centered
static int safetyOverrideStack[128]; // If things get weird make this number bigger!

// PIR Sensor (used to detect people/motion before the robot is initialized
/*
I got this PIR sensor for free, and it is fun,
but it doesn't do much for you when the robot is moving
and sending out IR signals with the Kinect/Xtion
and with the IR distance sensors. :)
It is cool though to have it watch for people and alert
ROS when someone is around. Then ROS can initialize the ArloBot,
start up the motors with a USB Relay Board
and start chasing them . . . or something.
*/
const int hasPIR = 1;  // Set this to 0 if you do not have PIR sensor mounted
const int PIRpin = 11; // Set this to the Activity Board pin that your PIR signal line is connected to

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
			char buf[51]; // A Buffer long enough to hold the longest line ROS may send.
			int count = 0;
			while (count < 51) {
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
            if(hasPIR == 1) {
                for(int i=0; i < 5; i++ ) { // 5 x 200ms pause = 1000 between updates
                    int PIRstate = input(PIRpin); // Check sensor (1) motion, (0) no motion
                    // Count positive hits and make a call:
                    if(PIRstate == 0) {
                      PIRhitCounter = 0;
                    } else {
                      PIRhitCounter++; // Increment on each positive hit
                    }
                    if(PIRhitCounter > personThreshhold) {
                      personDetected = 1;
                    } else {
                      personDetected = 0;
                    }
                    pause(200); // Pause 1/5 second before repeat
                  }
            } else {
                pause(1000); // Longer pauses when robot is uninitialized
            }
		}
	}
    
    // Start the sensor cog(s)
	cogstart(&pollPingSensors, NULL, pstack, sizeof pstack);
    
    if(hasGyro == 1) {
        // Initialize Gyro in the main program
      bus = i2c_newbus(gyroSCLpin, gyroSDApin, 0);        //New I2C bus SCL = Pin 1, SDA = Pin 0
      int n;
      n = i2c_out(bus, i2cAddr, ctrl3, 1, &cfg3, 1);
      n += i2c_out(bus, i2cAddr, ctrl4, 1, &cfg4, 1);
      n += i2c_out(bus, i2cAddr, ctrl1, 1, &cfg1, 1);
      // Make sure Gyro initialized and stall if it did not.
      if(n != 9)
      {
        print("Bytes should be 9, but was %d,", n);
        while(1); // This should just TELL ROS that there is no gyro available instead of stalling the program,
        // TODO:
        // OR have ROS tell us if we HAVE a gyro and only start this if we think we do.
        // That way the program works with or without a gyro
      }
      // Start Gyro polling in another cog  
        cogstart(&pollGyro, NULL, gyrostack, sizeof gyrostack);
    }
    
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

	// Listen for drive commands
	while (1) {

		/* TODO:
		 1. Should there should be code here to stop the motors if we go too long with no input from ROS?
		 */

		if (fdserial_rxReady(term) != 0) { // Non blocking check for data in the input buffer
			char buf[20]; // A Buffer long enough to hold the longest line ROS may send.
			int count = 0;
			while (count < 20) {
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
				double angularVelocityOffset = CommandedAngularVelocity * (trackWidth * 0.5);
                
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
                          }
                          ...
                    
                    So clearly we need to "normalize" the speed so that if one number is truncated,
                    the other is brought down the same amount, in order to accomplish the same turn
                    ratio at a slower speed!
                    Especially if the max speed is variable based on parameters within this code,
                    such as proximity to walls, etc.
                */
                if (CommandedVelocity > 0) {
                    if ((abd_speedLimit * distancePerCount) - fabs(angularVelocityOffset) < CommandedVelocity)
                        CommandedVelocity = (abd_speedLimit * distancePerCount) - fabs(angularVelocityOffset);
                } else {
                    if (-((abdR_speedLimit * distancePerCount) - fabs(angularVelocityOffset)) > CommandedVelocity)
                        CommandedVelocity = -((abdR_speedLimit * distancePerCount) - fabs(angularVelocityOffset));
                    }
                    
				double expectedLeftSpeed = CommandedVelocity - angularVelocityOffset;
				double expectedRightSpeed = CommandedVelocity + angularVelocityOffset;

				expectedLeftSpeed = expectedLeftSpeed / distancePerCount;
				expectedRightSpeed = expectedRightSpeed / distancePerCount;
                
                if (Escaping == 0) { // Don't fight with the Propeller escape code!
                    if (CommandedVelocity > 0 && safeToProceed == 1) {
                        //drive_speed(expectedLeftSpeed, expectedRightSpeed);
                        drive_speed(expectedLeftSpeed, expectedRightSpeed);
                    } else if (CommandedVelocity < 0 && safeToRecede == 1) {
                        drive_speed(expectedLeftSpeed, expectedRightSpeed);
                    } else if (CommandedVelocity == 0) {
                        // Rotate in place even if we are too close? Is this safe?!
                        drive_speed(expectedLeftSpeed, expectedRightSpeed);
                    }
                }
                // Using drive_rampStep does not appear to work with AMCL. It expects the robot to do what it is told, and
                // this doesn't give later commands Especially the command to slow to 0! The drive is left creepign after AMCL
                // tells it to stop!
                // NOTE: drive_setRampStep can be used to adjust the ramp rate.
                // NOTE: DO NOT use drive_ramp, as this will stall the program until the requested speed is reached!
                // Which is NOT what we want, since we may want to change the requested speed before we even reach it. ;)
                // The repeated "twist" statements provide the required "loop".
			}
		}
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
    1. I want to be able to visualize in RVIZ what the sensors are reporting. This with debugging
    situations where the robot gets stalled in doorways and such due to odd sensor readings from angled
    surfaces near the sides.
    2. I also want to use at least some of this for obstacle avoidance in AMCL.
    Note that I do not think that IR and PING data will be useful for gmapping, although it is possible.
    It is just too granular and non specific. It would be nice to be able to use the PING (UltraSonic) data
    to deal with mirrors and targets below the Kinect/Xtion, but I'm not sure how practical that is.
    */
    dprint(term, "o\t%.3f\t%.3f\t%.3f\t%.3f\t%.3f\t%.3f", X, Y, Heading, gyroHeading, V, Omega);
       for(int i=0; i < numberOfPINGsensors; i++ ) { // Loop through all of the sensors
        dprint(term, "\t%d\t%d", pingArray[i], irArray[i]);
        }
        dprint(term, "\n"); // Close line with a return.
    
    // Send a regular "status" update to ROS including information that does not need to be refreshed as often as the odometry.
    throttleStatus = throttleStatus + 1;
    if(throttleStatus > 9) {
       dprint(term, "s\t%d\t%d\t%d\t%d\t%d\n", safeToProceed, safeToRecede, Escaping, abd_speedLimit, abdR_speedLimit);
       throttleStatus = 0;
    }
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

void pollPingSensors(void *par) {
 while(1)                                    // Repeat indefinitely
  {
    for(int i=0; i < numberOfPINGsensors; i++ ) {
        pingArray[i] = ping_cm(firtPINGsensorPIN + i);
       // Check IR sensors on MCP3208 - Just checking 1 at a time instead of all each time. This works better!
       //for (int j=0; j < numberOfIRonMC3208; j++) {
          irArray[i] = mcp3208_IR_cm(i);
        //}
    }
  }
}

int mcp3208_IR_cm(int channel) {
const int MCP3208_dinoutPIN = 3;
const int MCP3208_clkPIN = 4;
const int MCP3208_csPIN = 2;
const float referenceVoltage = 5.0; // MCP3208 reference voltage setting. I use 5.0v for the 5.0v IR sensors from Parallax
  int mcp3208reading  = readADC(channel, MCP3208_dinoutPIN, MCP3208_clkPIN, MCP3208_csPIN);
  float mcp3208volts = (float)mcp3208reading * referenceVoltage / 4096.0;
  int mcp3208cm = 27.86 * pow(mcp3208volts, -1.15); // https://www.tindie.com/products/upgradeindustries/sharp-10-80cm-infrared-distance-sensor-gp2y0a21yk0f/
  return(mcp3208cm);
}

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

void pollGyro(void *par) {
while(1) {
    int ready = 0;                    //Wait until ready
    while (!ready)
    {
      i2c_in(bus, i2cAddr, status, 1, &ready, 1);
      ready = 1 & (ready >>= 3);
    }

    for(int i = 0; i < 6; i++)        //Get axis bytes
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

const int maxDistance = 70;
const int IRMaxDistance = 60; // Because IR is less reliable at long distances
const int haltDistance[6] = { 5, 10, 12, 10, 5, 12 };
const int minimumSpeed = 10;

void safetyOverride(void *par) {
// Each sensor needs to have its own response because of their various angles. We cannot use the same distance response for each, and we may want different max speeds and escape sequences.
int startSlowDownDistance[6] = { 10, 40, maxDistance, 40, 10, maxDistance };
int IRstartSlowDownDistance[6] = { 10, 40, IRMaxDistance, 40, 10, IRMaxDistance };
while(1) {
    int blockedSensor[6] = { 0 };
    int i, blocked = 0, minDistance = 255;
    // Walk PING Array to find blocked paths and halt immediately
    for( i=0; i < numberOfPINGsensors - 1; i++ ) { // -1 to ignore the one in back.
      if(pingArray[i] < startSlowDownDistance[i]) {
        if(pingArray[i] < minDistance)
          minDistance = pingArray[i];
        if(pingArray[i] < haltDistance[i]) {
            blocked = 1; // Use this to give the "all clear" later if it never gets set
            safeToProceed = 0; // Prevent main thread from setting any drive_speed
            blockedSensor[i] = 1; // Keep track of which sensors are blocked for intelligent escape sequences.
        }
      }
    }
    
    // Walk IR Array to find blocked paths and halt immediately
        // Check IR sensors on MCP3208
    for (i = 0; i < numberOfIRonMC3208 - 1; i++) { // -1 to ignore the one in back for now.
      if(irArray[i] < IRstartSlowDownDistance[i]) {
        if(irArray[i] < minDistance)
          minDistance = irArray[i];
        if(irArray[i] < haltDistance[i]) {
            blocked = 1; // Use this to give the "all clear" later if it never gets set
            safeToProceed = 0; // Prevent main thread from setting any drive_speed
            blockedSensor[i] = 1; // Keep track of which sensors are blocked for intelligent escape sequences.
        }
      }
    }
    
    // Check rear Sensor and just don't reverse, no escaping yet.
    if(pingArray[numberOfPINGsensors - 1] < haltDistance[numberOfPINGsensors - 1]) {
        safeToRecede = 0;
    } else if(irArray[numberOfIRonMC3208 - 1] < haltDistance[numberOfPINGsensors - 1]) {
        safeToRecede = 0;
    } else {
        safeToRecede = 1;
    }
    
    // Set backup speed limit
    int minRDistance = 255;
      if(pingArray[numberOfPINGsensors - 1] < startSlowDownDistance[numberOfPINGsensors - 1]) {
          minRDistance = pingArray[numberOfPINGsensors - 1];
          }
      if(irArray[numberOfIRonMC3208 - 1] < IRstartSlowDownDistance[numberOfPINGsensors - 1]) {
        if(irArray[numberOfIRonMC3208 - 1] < minRDistance) {
          minRDistance = irArray[numberOfIRonMC3208 - 1];
          }
      }

    // Reduce speed when we are close to an obstruction
    /*
    Ideas:
    1. A counter that requires a given speed to be arrived at X times before it is implemented.
    2. Same, but only for wide changes.
    3. A system by which it only changes X per round, so it has to change slowly, not rapidly
    Have to test each to see how it deals with sudden obstacles in the way,
    and with normal driving.
    */
    if( minDistance < maxDistance ) {
      // Set based on percentage of range
      int new_abd_speedLimit = (minDistance - haltDistance[1]) * (100 / (maxDistance - haltDistance[1]));
      // Limit maximum and minimum speed.
      if(new_abd_speedLimit < minimumSpeed) {
        new_abd_speedLimit = minimumSpeed;
      } else if(new_abd_speedLimit > 100) {
        new_abd_speedLimit = 100;
      }
      
      // Ramp and limit affect of random hits
      if(new_abd_speedLimit > abd_speedLimit) {
            abd_speedLimit = abd_speedLimit + 1;
        } else if(new_abd_speedLimit < abd_speedLimit) {
            abd_speedLimit = abd_speedLimit -1;
        }
    } else {
      abd_speedLimit = 100; // Full speed if all is well, let the built in speed ramping of drive_rampStep deal with the sudden increase
    }
    
    // Same for reverse speed limit
    // TODO: This code should probably be some sort of function instead of a cut and paste :)
    //abdR_speedLimit = 100;
    if( minRDistance < maxDistance ) {
      // Set based on percentage of range
      int newRLimit = (minRDistance - haltDistance[1]) * (100 / (maxDistance - haltDistance[1]));
      // Limit maximum and minimum speed.
      if(newRLimit < minimumSpeed) {
        newRLimit = minimumSpeed;
      } else if(newRLimit > 100) {
        newRLimit = 100;
      }

      // Ramp and limit affect of random hits
        if(newRLimit > abdR_speedLimit) {
            abdR_speedLimit = abdR_speedLimit + 1;
        } else if(newRLimit < abdR_speedLimit) {
            abdR_speedLimit = abdR_speedLimit -1;
        }
    } else {
      abdR_speedLimit = 100; // Full speed if all is well, let the built in speed ramping of drive_rampStep deal with the sudden increase
    }

    // If NO sensors are blocked, give the all clear!
    if(blocked == 0) {
        if(Escaping == 1) {// If it WAS unsafe before
            drive_speed(0,0); // return to stopped before giving control back to main thread
            }
        safeToProceed = 1;
        Escaping = 0; // Have fun!
    } else {
        Escaping = 1; // This will stop main thread from driving the motors.
        /* At this point we are blocked, so it is OK to take over control
           of the robot (safeToProceed == 0, so the main thread won't do anything),
           and it is safe to do work ignoring the need to slow down or stop
           because we know our position pretty well.
           HOWEVER, you will have to RECHECK distances yourself if you are going to move
           in this program location.
        */
        if (safeToRecede == 1) {
            // NOTE: This might cause a wiggle :)
            // TODO: Might need to add a pause on some of these to give them more time and avoid back and forthing with no progress.
            if(blockedSensor[2] == 1) {
                drive_speed(-minimumSpeed, -minimumSpeed); // back off slowly until we are clear. This needs more smarts.
            // Question: Should 0 and 4 have priority or 1 and 3?
            } else if(blockedSensor[1] == 1) { // block on sensor left of center
                drive_speed(-minimumSpeed, -(minimumSpeed * 2)); // Curve out to the right
            } else if(blockedSensor[3] == 1) { // blocked on sensor right of center
                drive_speed(-(minimumSpeed * 2), -minimumSpeed); // Curve out to the left
            } else if(blockedSensor[0] == 1) { // Blocked on far left side or near wall
                drive_speed(0, -minimumSpeed); // Turn out to the right slowly
            } else if(blockedSensor[4] == 1) { // Blocked on far right side or near wall
                drive_speed(-minimumSpeed, 0); // Turn out to the left slowly
            }
        } else {
            drive_speed(0,0);
        }
    }

    pause(1); // This throttles the transition speed of everything in this cog.
    /* Imagine the functions here, and how many times they have to iterate to achieve
       a given setting, and multiply it by this to determine the maximum transition
       speed.
    */

        }
}
