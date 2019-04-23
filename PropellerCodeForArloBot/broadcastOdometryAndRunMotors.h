#ifndef BROADCAST_ODOMETRY_H
#define BROADCAST_ODOMETRY_H

/* About when and why to use static and/or volatile variables on Propeller:
 https://learn.parallax.com/propeller-c-library-studies/functions-multiple-files-sharing-variables
 Ignore advice that is not directly about the Propeller CPU.
 */
static volatile int broadcastSpeedLeft = 0;
static volatile int broadcastSpeedRight = 0;

// These are global so we can read and set them from ROS on a restart if we want
// to.
static float Heading = 0.0;
static float X = 0.0;
static float Y = 0.0;
void broadcastOdometryAndRunMotors(void *par);
static int fstack[256]; // If things get weird make this number bigger!

void broadcastOdometryAndRunMotors(void *par) {
  dprint(term, "%c%ceInitializing Motors%c", START_MARKER, SPECIAL_BYTE,
         END_MARKER);
  // ALL Interaction with the DHB-10 happens in here!
  // For DHB-10 Interaction See drive_speed.c for example code
  // Any other cog/function that needs to affect the robot's motors will set
  // variables that are read in this function.
  char dhb10_reply[DHB10_LEN];

  char s[32]; // Hold strings converted for sending to DHB-10
  memset(s, 0, 32);
  char *reply = dhb10_reply;

  // Halt motors and reset counts to 0 in case they are moving somehow
  dhb10_com("GOSPD 0 0\r");
  pause(dhb10OverloadPause);
  dhb10_com("RST\r");
  pause(dhb10OverloadPause);
  // int acc = DHB10_ACC;
  // int acc = DHB10_MAX_ACC;
  sprint(s, "ACC %d\r", DHB10_ACC);
  dhb10_com(s);
  pause(dhb10OverloadPause);

  // For Odometry
  int ticksLeft = 0;
  int ticksRight = 0;
  int ticksLeftOld;
  int ticksRightOld;
  int speedLeft;
  int speedRight;
  int deltaTicksLeft;
  int deltaTicksRight;
  int newLeftSpeed = 0;
  int newRightSpeed = 0;
  int oldLeftSpeed = 0;
  int oldRightSpeed = 0;
  float deltaDistance;
  float deltaTheta;
  float deltaX;
  float deltaY;
  float V;
  float Omega;
  float leftMotorPower = DEFAULT_MOTOR_ADC_VOLTAGE;
  float rightMotorPower = DEFAULT_MOTOR_ADC_VOLTAGE;

  int dt = CLKFREQ / 10;
  // Operates once per every 100 ms
  // https://lamestation.atlassian.net/wiki/display/SPIN/CLKFREQ
  int t = CNT;

  dprint(term, "%c%ceOdometry Cog Started%c", START_MARKER, SPECIAL_BYTE,
         END_MARKER);
  uint8_t i;
  while (true) {
    if (CNT - t > dt && propellerBoardInitialized) {
      // For debugging and testing
      // dprint(term, "%c%ce dt: %d CNT: %d t: %d %d CLKFREQ: %d %c",
      // START_MARKER, SPECIAL_BYTE, dt, CNT, t, CNT - t, CLKFREQ, END_MARKER);
      t += dt;

      oldLeftSpeed = newLeftSpeed;
      oldRightSpeed = newRightSpeed;

      newLeftSpeed = broadcastSpeedLeft;
      newRightSpeed = broadcastSpeedRight;

      // Send resulting speed to wheels IF it is different from last time
      if (newLeftSpeed != oldLeftSpeed || newRightSpeed != oldRightSpeed) {
        dprint(term, "%c%ceGOSPD: previous %d,%d new %d,%d%c", START_MARKER,
               SPECIAL_BYTE, oldLeftSpeed, oldRightSpeed, newLeftSpeed,
               newRightSpeed, END_MARKER); // For Debugging
        sprint(s, "GOSPD %d %d\r", newLeftSpeed, newRightSpeed);
        dhb10_com(s);
        pause(dhb10OverloadPause);
      }

      // Broadcast Odometry
      /* Some of the code below came from Dr. Rainer Hessmer's robot.pde
         The rest was heavily inspired/copied from here:
      http://forums.parallax.com/showthread.php/154963-measuring-speed-of-the-ActivityBot?p=1260800&viewfull=1#post1260800
      */
      ticksLeftOld = ticksLeft;
      ticksRightOld = ticksRight;

#ifdef encoderConnectedToPropeller
      // Use encoder values from Propeller board
      ticksLeft = left_ticks;
      ticksRight = right_ticks;
#else
      // Else use DHB10 for DIST data
      int64_t dhb10_ticksLeft, dhb10_ticksRight;
      reply = dhb10_com("DIST\r");
      if (*reply == '\r') {
        // Wrong response
        dhb10_ticksLeft = 0;
        dhb10_ticksRight = 0;
      } else {
        sscan(reply, "%d%d", &dhb10_ticksLeft, &dhb10_ticksRight);
      }
      ticksLeft = dhb10_ticksLeft;
      ticksRight = dhb10_ticksRight;
#endif

      // For Debugging:
      // dprint(term, "d\tDIST\t%d\t%d\n", ticksLeft, ticksRight);

      pause(dhb10OverloadPause);

      reply = dhb10_com("SPD\r");
      if (*reply == '\r') {
        speedLeft = 0;
        speedRight = 0;
      } else {
        sscan(reply, "%d%d", &speedLeft, &speedRight);
      }

#ifdef hasGyro
      if (speedLeft == speedRight) {
        isRotating = 0;
      } else {
        isRotating = 1;
      }
#endif

      deltaTicksLeft = ticksLeft - ticksLeftOld;
      deltaTicksRight = ticksRight - ticksRightOld;
      deltaDistance =
          0.5f * (float)(deltaTicksLeft + deltaTicksRight) * distancePerCount;
      deltaX = deltaDistance * cos(Heading);
      deltaY = deltaDistance * sin(Heading);

      deltaTheta =
          (deltaTicksRight - deltaTicksLeft) * distancePerCount / trackWidth;
      Heading += deltaTheta;
      if (Heading > PI)
        Heading -= 2 * PI;
      if (Heading < -PI)
        Heading += 2 * PI;
      X += deltaX;
      Y += deltaY;

      // http://webdelcire.com/wordpress/archives/527
      V = ((speedRight * distancePerCount) + (speedLeft * distancePerCount)) /
          2;
      Omega =
          ((speedRight * distancePerCount) - (speedLeft * distancePerCount)) /
          trackWidth;

      // Odometry for ROS
      /*
        I sending ALL of the proximity data (IR and PING sensors) to ROS
        over the "odometry" line, since it is real time data which is just as
        important as the odometry, and it seems like it would be faster to send
        and deal with one packet per cycle rather than two.

        In the propeller node I will convert this to fake laser data.
        I have two goals here:
        1. I want to be able to visualize in RVIZ what the sensors are
        reporting. This will help with debugging situations where the robot gets
        stalled in doorways and such due to odd sensor readings from angled
        surfaces near the sides.
        2. I also want to use at least some of this for obstacle avoidance in
        AMCL. Note that I do not think that IR and PING data will be useful for
        gmapping, however introductin the PING and IR data into AMCL as
        obstacles helps with path planning around obstacles that are not seen by
        the 3D Kinect/ASUS camera.
        */

// Check Motor Power
#ifdef hasMotorPowerMonitorCircuit
      // The input numbers for adc_init are printed right on the Activity
      // Board.
      adc_init(21, 20, 19, 18);
      leftMotorPower = adc_volts(LEFT_MOTOR_ADC_PIN);
      rightMotorPower = adc_volts(RIGHT_MOTOR_ADC_PIN);
#endif

      // Fill odometryData.dataStruct with Data
      odometryData.dataStruct.X = X;
      odometryData.dataStruct.Y = Y;
      odometryData.dataStruct.Heading = Heading;
      odometryData.dataStruct.gyroHeading = gyroHeading;
      odometryData.dataStruct.V = V;
      odometryData.dataStruct.Omega = Omega;
      odometryData.dataStruct.safeToProceed = safeToProceed;
      odometryData.dataStruct.safeToRecede = safeToRecede;
      odometryData.dataStruct.Escaping = Escaping;
      odometryData.dataStruct.abd_speedLimit = abd_speedLimit;
      odometryData.dataStruct.abdR_speedLimit = abdR_speedLimit;
      odometryData.dataStruct.minDistanceSensor = minDistanceSensor;
      odometryData.dataStruct.leftMotorPower = leftMotorPower;
      odometryData.dataStruct.rightMotorPower = rightMotorPower;
      odometryData.dataStruct.cliff = cliff;
      odometryData.dataStruct.floorO = floorO;

      uint8_t sensorDataPosition = 0;
      for (i = 0; i < NUMBER_OF_PING_SENSORS; i++) {
        odometryData.dataStruct.sensorData[sensorDataPosition] = pingArray[i];
        sensorDataPosition++;
      }

      for (i = 0; i < NUMBER_OF_IR_SENSORS; i++) {
        odometryData.dataStruct.sensorData[sensorDataPosition] = irArray[i];
        sensorDataPosition++;
      }

#ifdef hasFloorObstacleSensors
      for (i = 0; i < NUMBER_OF_FLOOR_SENSORS; i++) {
        odometryData.dataStruct.sensorData[sensorDataPosition] = floorArray[i];
        sensorDataPosition++;
      }
#endif

#ifdef hasButtons
      for (i = 0; i < NUMBER_OF_BUTTON_SENSORS; i++) {
        odometryData.dataStruct.sensorData[sensorDataPosition] = buttonArray[i];
        buttonArray[i] = 0;
        sensorDataPosition++;
      }
#endif

#ifdef hasLEDs
      for (i = 0; i < NUMBER_OF_LEDS; i++) {
        odometryData.dataStruct.sensorData[sensorDataPosition] = ledArray[i];
        sensorDataPosition++;
      }
#endif

      // Add/update checksum in struct for outgoing data
      odometryData.dataStruct.dataCheckSum =
          calculateChecksum(odometryData.inputArray, 0, odometryDataLength - 1);
      // Send data
      sendOverSerial(odometryData.inputArray, odometryDataLength,
                     odometryDataCharacter);
    } else {
      // Without a pause() line here, this cog locks up.
      // The only issue is to make sure the sleep is short enough to not
      // interfeer with the clock based timing.
      // clkfreq / 10 = Clock ticks per 0.1 seconds (100 ms)
      // So a pause of 1 or even 10 should leave plenty of granularity.
      pause(1);
    }
  }
}

#endif /* BROADCAST_ODOMETRY_H */
