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
#include "fdserial.h"
#include "ping.h"                             // Include ping header
/*
http://forums.parallax.com/showthread.php/154274-The-quot-Artist-quot-robot?p=1277133&viewfull=1#post1277133
"The most impressive is that we can use the same code as the ActivityBot examples, replacing only the library’s name. So everyone who has a big-size Robot, similar to The Artist (like Arlo, Eddie or any other) must only change the library “abdrive.h” with the “arlodrive.h”. So we can take all the advantages form the pre-written code for the ActivityBot!'
http://www.parallax.com/news/2013-12-23/run-activitybot-c-code-arlo
http://forums.parallax.com/showthread.php/152636-Run-ActivityBot-C-Code-in-the-Arlo!?p=1230592&posted=1#post1230592
*/
#include "arlodrive.h"

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

// for Ping Sensors
// Name them based on the center of the direction
// in which they point in radians
// (radians because that is what sensor_msgs/LaserScan uses)
static int pingRange0 = 0;
void pollPingSensors(void *par); // Use a cog to fill range variables with ping distances
static int pstack[256]; // If things get weird make this number bigger!

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

int main() {

	simpleterm_close(); // Close simplex serial terminal
	term = fdserial_open(31, 30, 0, 115200); // Open Full Duplex serial connection

	/* Wait for ROS to give us the robot parameters,
	 broadcasting 'i' until it does to tell ROS that we
	 are ready */
	int robotInitialized = 0; // Do not compute odometry until we have the trackWidth and distancePerCount
    
    // For Debugging without ROS:
    /*
    // See encoders.yaml for most up to date values
    trackWidth = 0.403000; // from measurement and then testing
    distancePerCount = 0.006760; // http://forums.parallax.com/showthread.php/154274-The-quot-Artist-quot-robot?p=1271544&viewfull=1#post1271544
    robotInitialized = 1;
    */
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
  bus = i2c_newbus(5, 4, 0);        //New I2C bus SCL = P5, SDA = P4
  int n;
  n = i2c_out(bus, i2cAddr, ctrl3, 1, &cfg3, 1);
  n += i2c_out(bus, i2cAddr, ctrl4, 1, &cfg4, 1);
  n += i2c_out(bus, i2cAddr, ctrl1, 1, &cfg1, 1);
  // Make sure Gyro initialized and stall if it did not.
  if(n != 9)
  {
    print("Bytes should be 9, but was %d,", n);
    while(1);
  }
  // Start Gyro polling in another cog  
	cogstart(&pollGyro, NULL, gyrostack, sizeof gyrostack);

	// Now initialize the Motors
	// abdrive settings:
	drive_speed(0, 0);                     // Start servos/encoders cog
	//drive_setRampStep(10);              // Set ramping at 10 ticks/sec per 20 ms
	// TODO Do we need to adjust ramping? Perhaps this should be something we can modify on the ROS side and send?

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
				double angularVelocityOffset = 0.5 * CommandedAngularVelocity * trackWidth;
				double expectedLeftSpeed = CommandedVelocity - angularVelocityOffset;
				double expectedRightSpeed = CommandedVelocity + angularVelocityOffset;

				expectedLeftSpeed = expectedLeftSpeed / distancePerCount;
				expectedRightSpeed = expectedRightSpeed / distancePerCount;

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
    dprint(term, "o\t%.3f\t%.3f\t%.3f\t%.3f\t%.3f\t%.3f\t%d\n", X, Y, Heading, gyroHeading, V, Omega, pingRange0);
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
    pingRange0 = ping_cm(8);                 // Get cm distance from Ping)))
    pause(200);                               // Wait 1/5 second
    /*
    http://forums.parallax.com/showthread.php/111215-multiple-pings?highlight=ping+round+trip+time
"The echos from one PING could confuse the others if they were operating simultaneously. You really need to trigger only one at a time and allow a little time for the echos to die down before triggering another PING even if they're facing in completely different directions.
I don't know what would be ideal for a delay, but the maximum sound round-trip time window for the PING))) is 18.5ms and I'd wait several times that, maybe 50 to 100ms."
    */
  }

}

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