/*
 ActivityBot (AB) code to connect to Dr. Rainer Hessmer's
 ROS with Arduino project: https://code.google.com/p/drh-robotics-ros/
 This code is largely a sloppy "conversion" of Dr. Hessmer's robot.pde for the Arduino:
 https://code.google.com/p/drh-robotics-ros/source/browse/trunk/Arduino/Robot/Robot.pde
 Dr. Hessmer's blog explaining how his code and robot works is here:
 http://www.hessmer.org/blog/2010/11/21/sending-data-from-arduino-to-ros/

 Please also see these sites which helped me tremendously with the formulas:
 http://www.seattlerobotics.org/encoder/200610/article3/IMU%20Odometry,%20by%20David%20Anderson.htm
 http://webdelcire.com/wordpress/archives/527
 And of course the entire point of this is to interface with ROS, so read about everything ROS here:
 http://wiki.ros.org/
 All code here heavily borrowed from everywhere code can be found! :)
 See "Serial Testing for ROS ActivityBot 1-4" (which are not on GitHub) for previous versions and explanations.
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

fdserial *term;

// Robot description: We will get this from ROS so that it is easier to tweak between runs without reloading the AB EEPROM.
// http://learn.parallax.com/activitybot/calculating-angles-rotation
static double distancePerCount = 0.0; // See encoders.yaml to set or change this value
static double trackWidth = 0.0; // See encoders.yaml to set or change this value

const char delimiter[2] = ","; // Delimiter character for incoming messages from the ROS Python script

// For Odometry
int ticksLeft, ticksRight, ticksLeftOld, ticksRightOld;
static double Heading = 0.0, X = 0.0, Y = 0.0;
static int speedLeft, speedRight;

void getTicks();
void displayTicks();
//void drive_getSpeedCalc(int *left, int *right); // Built into arlodrive.h, not abdrive.h

void broadcastOdometry(void *par); // Use a cog to broadcast Odometry to ROS continuously
static int fstack[256]; // If things get weird make this number bigger!

// for Proximity (PING & IR) Sensors
int pingArray[6];
int irArray[6];
void pollPingSensors(void *par); // Use a cog to fill range variables with ping distances
static int pstack[128]; // If things get weird make this number bigger!
const int MCP3208_dinoutPIN = 3;
const int MCP3208_clkPIN = 4;
const int MCP3208_csPIN = 2;
const float referenceVoltage = 5.0; // MCP3208 reference voltage setting. I use 5.0v for the 5.0v IR sensors from Parallax
const int IRsamples = 3;
const int numberOfIRonMC3208 = 6; // Number of IR Sensors on the MCP3208 ADC to read
const int firtPINGsensorPIN = 5; // Which pin the first PING sensor is on
const int numberOfPINGsensors = 6; // Number of PING sensors, we assume the are consecutive
int mcp3208_IR_cm(int); // Function to get distance in CM from IR sensor using MCP3208
//int adc_IR_cm(int); // Function to get distance in CM from IR sensor using Activty Board built in ADC

// For Gyroscope - Declare everything globally
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
static int gyrostack[256]; // If things get weird make this number bigger!

// For "Safety Override"
static int safeToProceed = 0;

// Cog
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

    trackWidth = 0.403000; // from measurement and then testing
    distancePerCount = 0.006760; // http://forums.parallax.com/showthread.php/154274-The-quot-Artist-quot-robot?p=1271544&viewfull=1#post1271544
    robotInitialized = 1;

    // Comment out above lines for use with ROS
    
	while (robotInitialized == 0) {
		dprint(term, "i\t0\n"); // Request Robot distancePerCount and trackWidth NOTE: Python code cannot deal with a line with no divider characters on it.
		pause(10); // Give ROS time to respond, but not too much or we bump into other stuff that may be coming in from ROS.
		if (fdserial_rxReady(term) != 0) { // Non blocking check for data in the input buffer
			char buf[20]; // A Buffer long enough to hold the longest line ROS may send.
			int count = 0;
			while (count < 20) {
				buf[count] = readChar(term);
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
				if (trackWidth > 0.0 && distancePerCount > 0.0)
					robotInitialized = 1;
			}
		} else {
			pause(500); // Longer pauses when robot is uninitialized
		}
	}
    
    // Start the sensor cog(s)
	cogstart(&pollPingSensors, NULL, pstack, sizeof pstack);

    // Initialize Gyro in the main program
  bus = i2c_newbus(1, 0, 0);        //New I2C bus SCL = Pin 1, SDA = Pin 0
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
    
	// Now initialize the Motors
	// abdrive settings:
	drive_speed(0, 0);                     // Start servos/encoders cog
    drive_setMaxSpeed(abd_speedLimit);
	//drive_setRampStep(10);              // Set ramping at 10 ticks/sec per 20 ms
	// TODO Do we need to adjust ramping? Perhaps this should be something we can modify on the ROS side and send?

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
		 2. There should be a way to reset the odometry to 0 remotely, for when we pick up the robot and start over, so we don't have to press the reset button on the AB every time.
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
                /* From turtlebot_node.py, note the * 1000 is to convert to mm, but we did that in the trackWidth variable
                ts  = msg.linear.x * 1000 # m -> mm
                tw  = msg.angular.z  * (robot_types.ROBOT_TYPES[self.robot_type].wheel_separation / 2) * 1000 
                # Prevent saturation at max wheel speed when a compound command is sent.
                if ts > 0:
                    ts = min(ts,   MAX_WHEEL_SPEED - abs(tw))
                else:
                    ts = max(ts, -(MAX_WHEEL_SPEED - abs(tw)))
                self.req_cmd_vel = int(ts - tw), int(ts + tw)
                */
                
                // u = - 0.546009/154.096313 (Writing to serial port: s,0.519,2.594)
                // distancePerCount = 0.006760 Arlobot default currently
                
                // Prevent saturation at max wheel speed when a compound command is sent.
                if (CommandedVelocity > 0) {
                    if ((abd_speedLimit * distancePerCount) - fabs(angularVelocityOffset) < CommandedVelocity)
                        CommandedVelocity = (abd_speedLimit * distancePerCount) - fabs(angularVelocityOffset);
                } else {
                    if (-((abd_speedLimit * distancePerCount) - fabs(angularVelocityOffset)) > CommandedVelocity)
                        CommandedVelocity = -((abd_speedLimit * distancePerCount) - fabs(angularVelocityOffset));
                    }
                    
				double expectedLeftSpeed = CommandedVelocity - angularVelocityOffset;
				double expectedRightSpeed = CommandedVelocity + angularVelocityOffset;

				expectedLeftSpeed = expectedLeftSpeed / distancePerCount;
				expectedRightSpeed = expectedRightSpeed / distancePerCount;
                
                /* Speed normalization:
                What if the calculated speed is above the max speed in arlodrive.c?
                Then it will set the speed to the max,
                but then our left/right ratio will be wrong, say it was supposed to be 15/110,
                and max was 100, it turns into 15/100!
                From arlodrive.c:
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
                
                First I want to know what typical numbers are here without normalization.
                roslaunch turtlebot_teleop keyboard_teleop.launch
                "currently:      speed 0.2       turn 1"
                Here is what I get when I press this key and hold it until it maxes out:
                (Note that it does "ramp up")
                i = 29.585800/29.585800
                , = -29.585800/-29.585800
                j = -29.807693/29.807693
                l = 29.807693/-29.807693
                u = -0.221893/59.393490
                o = 59.393490/-0.221893
                m = 0.221893/-59.393490
                . = -59.393490/+0.221893
                q a few times to get:
                "currently:      speed 0.51874849202     turn 2.5937424601"
                i = 76.775146
                u = - 0.546009/154.096313 (Writing to serial port: s,0.519,2.594)
                
                Note these settings:
                 /turtlebot_teleop_keyboard/scale_angular: 1.5
                 /turtlebot_teleop_keyboard/scale_linear: 0.5
                */
                
                // For debugging: Note, this will cause trouble because of collisions on access to the serial port with broadcastOdometry
                //dprint(term, "d\t%f\t%f\n", expectedLeftSpeed, expectedRightSpeed);

				if (safeToProceed == 1)
                    drive_speed(expectedLeftSpeed, expectedRightSpeed);
                // Should this use drive_rampStep instead? Allowing the robot to ramp up/down to the requested speed?
                // The repeated "twist" statements provide the required "loop".
                // NOTE: drive_setRampStep can be used to adjust the ramp rate.
                // NOTE: DO NOT use drive_ramp, as this will stall the program until the requested speed is reached!
                // Which is NOT what we want, since we may want to change the requested speed before we even reach it. ;)
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
    dprint(term, "o\t%.3f\t%.3f\t%.3f\t%.3f\t%.3f\t%.3f\t%d\n", X, Y, Heading, gyroHeading, V, Omega, pingArray[2]);
    // For Debugging in Terminal mode
/*
    dprint(term, "Debug:%c\n", CLREOL);
    int i;
    for( i=0; i < numberOfPINGsensors; i++ ) {
        dprint(term, "Sensor %d: %d %d%c\n", i, pingArray[i], irArray[i], CLREOL);
        }
    dprint(term, "safeToProceed: %d abd_speedLimit: %d%c\n", safeToProceed, abd_speedLimit, CLREOL);
    dprint(term, "X: %.3f Y: %.3f V: %.3f Omega: %.3f%c\n", X, Y, V, Omega, CLREOL);
    dprint(term, "Heading: %.3f gyroHeading: %.3f%c\n", X, Y, CLREOL);
    dprint(term, "%c", HOME);                            // Cursor -> top-left "home"
*/
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

    // For Activity Board built in ADC
  //adc_init(21, 20, 19, 18);                   // CS=21, SCL=20, DO=19, DI=18 // Same on all Activity Boards, actually printed on the board!

  float adc_0v, adc_1v;
  int adc_0cm, adc_1cm;

    const int betweenSensorPollDelay = 100; // Time to wait between firing PING sensors to avoid interferance

  while(1)                                    // Repeat indefinitely
  {
    int i, j;
    for( i=0; i < numberOfPINGsensors; i++ ) {
    pingArray[i] = ping_cm(firtPINGsensorPIN + i);
    //dprint(term, "d\t%d\t%d\n", firtPINGsensorPIN + i, pingArray[i]); // For debugging, will collide with Odometry output sometimes
    // Check IR sensors on MCP3208
    for (j = 0; j < numberOfIRonMC3208; j++) {
    irArray[j] = mcp3208_IR_cm(j);
    }
    }
  }
}

int mcp3208_IR_cm(int channel) {

  	int i, j, k = 0, goodSamples = 0;
	for (i=0;i<IRsamples;i++) {
    //adc0 = readADC(0, 1, 2, 3); // Pull 1 reading
    j = readADC(channel, MCP3208_dinoutPIN, MCP3208_clkPIN, MCP3208_csPIN);
    if(j > 324) { // Anything lower is probably beyond the sensor's range
      k = k + j;
      goodSamples = goodSamples + 1;
    }
  }
  int mcp3208reading = 300; // Set devault to 300, Consider 300, aka. 88cm Out of Range! (Anything over 80 really)
  if(goodSamples > 0) {
    mcp3208reading  = k/goodSamples;
  }
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

const int startSlowDownDistance = 70;
const int IRstartSlowDownDistance = 30; // Because IR is jumpy at long distances
const int haltDistance = 10;

void safetyOverride(void *par) {
while(1) {
    int i, blocked = 0, minDistance = 255;
    // Walk PING Array to find blocked paths and halt immediately
    for( i=0; i < numberOfPINGsensors - 1; i++ ) { // -1 to ignore the one in back for now.
      if(pingArray[i] < startSlowDownDistance) {
        if(pingArray[i] < minDistance)
          minDistance = pingArray[i];
        if(pingArray[i] < haltDistance) {
            blocked = 1; // Use this to give the "all clear" later if it never gets set
            safeToProceed = 0; // Prevent main thread from setting any drive_speed
        }
      }
    }
    
    // Walk IR Array to find blocked paths and halt immediately
        // Check IR sensors on MCP3208
    for (i = 0; i < numberOfIRonMC3208 - 1; i++) { // -1 to ignore the one in back for now.
      if(irArray[i] < IRstartSlowDownDistance) {
        if(irArray[i] < minDistance)
          minDistance = irArray[i];
        if(irArray[i] < haltDistance) {
            blocked = 1; // Use this to give the "all clear" later if it never gets set
            safeToProceed = 0; // Prevent main thread from setting any drive_speed
        }
      }
    }

    // Reduce speed when we are close to an obstruction
    // TODO: This needs some smoothing, to avoid random and rapid jumps between speeds, although it
    // may be that sometimes a jump is needed, it just needs to stand the test of time.
    /*
    Ideas:
    1. A counter that requires a given speed to be arrived at X times before it is implemented.
    2. Same, but only for wide changes.
    3. A system by which it only changes X per round, so it has to change slowly, not rapidly
    Have to test each to see how it deals with sudden obstacles in the way,
    and with normal driving.
    */
    if( minDistance < startSlowDownDistance ) {
      int new_abd_speedLimit = (minDistance - haltDistance) * (100 / (startSlowDownDistance - haltDistance));
      if(new_abd_speedLimit < 5) {
        abd_speedLimit = 5;
      } else if(new_abd_speedLimit > 100) {
        abd_speedLimit = 100;
      } else {
        /* Only increment by 1 per iteration to smooth out transitions,
           and noise. Maximum transition speed is the pause at the bottom
           of this cog function * the transition amount.
        */
        
        if(new_abd_speedLimit > abd_speedLimit) {
            abd_speedLimit = abd_speedLimit + 1;
        } else if(new_abd_speedLimit < abd_speedLimit) {
            abd_speedLimit = abd_speedLimit -1;
        }
      }
    } else {
      abd_speedLimit = 100;
    }
    
    // If NO sensors are blocked, give the all clear!
    if(blocked == 0) {
        if(safeToProceed == 0) {// If it WAS unsafe before
            drive_speed(0,0); // return to stopped before giving control back to main thread
            }
        safeToProceed = 1;
    } else {
        /* At this point we are blocked, so it is OK to take over control
           of the robot (safeToProceed == 0, so the main thread won't do anything),
           and it is safe to do work ignoring the need to slow down or stop
           because we know our position pretty well.
           HOWEVER, you will have to RECHECK distances yourself if you are going to move
           in this program location.
        */
        drive_speed(-10,-10); // back off slowly until we are clear. This needs more smarts.
    }

    pause(1); // This throttles the transition speed of everything in this cog.
    /* Imagine the functions here, and how many times they have to iterate to achieve
       a given setting, and multiple it by this to determine the maximum transition
       speed.
    */

        }
}
