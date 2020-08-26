#ifndef POLL_GYRO_H
#define POLL_GYRO_H

/*
   NOTE About Gyro module:
   Currently this code makes NO use of the gyro. The Odometry from the ArloBot's
   wheel encoders is excellent! The only thing I do with the gyro is send the
   data to ROS and display it on the web interface. At some point a ROS node
   could use that data to detect a serious issue, like the robot being picked up
   or being stuck. As it is though, mapping works very well off of
   the Odometry without using the gyro data.
   */
const unsigned char i2cAddr = 0x69; // I2C Gyro address
// L3G4200D register addresses & commands.
// See device data sheet section 7 for more info.
// unsigned char devId = 0x0f;        //Device ID
const unsigned char ctrl1 = 0x20; // Control reg1
const unsigned char cfg1 =
    0b00011111; // 100 hz, 25 cutoff, power up, axes enabled
// unsigned char ctrl2 = 0x21;
const unsigned char ctrl3 = 0x22;
const unsigned char cfg3 = 0b00001000; // Enable data poling (I2_DRDY)
const unsigned char ctrl4 = 0x23;
const unsigned char cfg4 = 0b10000000; // Block until read, big endian
const unsigned char status = 0x27;
const unsigned char xL =
    0x28; // Reg for x low byte - Next 5 bytes xH, yL, yH, zL, xH
// unsigned char reply;                //Single byte reply
static char xyz[6]; // XYZ dat array
// int gyroXvel, gyroYvel, gyroZvel;                       //Axis variables
static uint8_t gyroZvel;
i2c *bus; // Declare I2C bus
// Create a cog for polling the Gyro
void pollGyro(
    void *par); // Use a cog to fill range variables with ping distances
static uint8_t gyrostack[128]; // If things get weird make this number bigger!
static uint8_t isRotating = 0;

void pollGyro(void *par) {
  int ready = 0; // Wait until ready
  double deltaGyroHeading;
  int i;
  while (1) {
    while (!ready) {
      i2c_in(bus, i2cAddr, status, 1, &ready, 1);
      ready = 1 & (ready >>= 3);
    }

    for (i = 0; i < 6; i++) // Get axis bytes
    {
      int regAddr = xL + i;
      i2c_in(bus, i2cAddr, regAddr, 1, &xyz[i], 1);
    }

    // Bytes to int in Degrees Per Second (dps)
    //"Dividing by 114 reduces noise"
    // http://www.parallax.com/sites/default/files/downloads/27911-L3G4200D-Gyroscope-Application-Note.pdf
    // 1 radian/second [rad/s] = 57.2957795130824 degree/second [°/s]
    // gyroXvel = (int) (short) ((xyz[1] << 8) + xyz[0]) / 114; // Perhaps use
    // later to detect tipping? gyroYvel = (int) (short) ((xyz[3] << 8) +
    // xyz[2]) / 114; // Perhaps use later to detect tipping?
    gyroZvel = (int)(short)((xyz[5] << 8) + xyz[4]) / 114;

    // If Gyro is running at 100Hz then time between readings should be 10
    // milliseconds
    deltaGyroHeading =
        0.01 * gyroZvel *
        2; // I'm not sure why I have to multiply by two, but I do.
    deltaGyroHeading = deltaGyroHeading * PI / 180.0; // Convert to Radians

    // Discard small variations when robot is not rotating to eliminate
    // stationary drift Maybe this should be ANY time that speedLeft ==
    // speedRight? Then straight lines would stay straight, since ActivityBot
    // appears to travel VERY good straight lines, but they seem to wobble in
    // RVIZ at the moment.
    if (isRotating == 0) {
      if (deltaGyroHeading <
          0.01) { // But accept large changes in case the robot is bumped or
                  // moved. Adjust as needed
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
  }
}

#endif /* POLL_GYRO_H */
