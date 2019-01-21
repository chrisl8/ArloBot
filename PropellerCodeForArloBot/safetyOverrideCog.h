#ifndef SAFETY_OVERRIDE_COG_H
#define SAFETY_OVERRIDE_COG_H

// Config Variables: These are rarely changing settings,
// that are given to us by ROS in "settings" packets,
// and confirmed in "config" packets.
// NOTE: Setting 'pluggedIn' to true by default prevents robot from moving,
// before the Python Serial Interface connects to it!
static volatile bool pluggedIn = true;
static volatile bool ignoreProximity = false;
static volatile bool ignoreCliffSensors = false;
static volatile bool ignoreFloorSensors = false;
static volatile bool ignoreIRSensors = false;

// Telemetry Variables: These change constantly based on
// input from sensors, and are relayed to ROS
// via the "odometry" packets
static volatile bool safeToProceed = false;
static volatile bool safeToRecede = false;
static volatile bool cliff = false;
static volatile bool floorO = false;
static volatile bool Escaping = false;
static volatile int escapeLeftSpeed = 0;
static volatile int escapeRightSpeed = 0;
static volatile uint8_t minDistanceSensor = 0;

// Use a cog to squelch incoming commands and perform safety
// procedures like halting, backing off, avoiding cliffs,
// calling for help, etc.
// This can use proximity sensors to detect obstacles (including people) and
// cliffs This can use the gyro to detect tipping This can use the gyro to
// detect significant heading errors due to slipping wheels when an obstacle is
// encountered or high centered
void safetyOverride(void *par);
static int
    safetyOverrideStack[128]; // If things get weird make this number bigger!

void setEscapeSpeeds(int left, int right) {
  escapeLeftSpeed = left;
  escapeRightSpeed = right;
}

void safetyOverride(void *par) {
  dprint(term, "%c%ceSafetyOverride Cog Started%c", START_MARKER, SPECIAL_BYTE,
         END_MARKER);

  int increaseThrottleRamp = 0;
  int decreaseThrottleRamp = 0;
  // Declare all variables up front so they do not have to be created in the
  // loop, only set. This may or may not improve performance.
  bool blockedSensor[NUMBER_OF_PING_SENSORS] = {false};
  uint8_t i;
  bool blockedF = false, blockedR = false, foundCliff = false,
       floorObstacle = false, pleaseEscape = false;
  long minDistance, minRDistance, newSpeedLimit;
  while (1) {
    if (ignoreProximity == false) {
      // Reset blockedSensor array to all zeros.
      memset(blockedSensor, false, sizeof(blockedSensor));
      blockedF = false;
      blockedR = false;
      pleaseEscape = false;
      minDistance = 255;
      minRDistance = 255;

#ifdef hasCliffSensors
      foundCliff = false;
      if (ignoreCliffSensors == false) {
        // Check Cliff Sensors first
        for (i = FIRST_CLIFF_SENSOR;
             i < FIRST_CLIFF_SENSOR + NUMBER_OF_CLIFF_SENSORS; i++) {
          if (irArray[i] > FLOOR_DISTANCE) {
            // Set the global 'cliff' variable so we can see this in ROS.
            cliff = true;
            safeToProceed =
                false; // Prevent main thread from setting any drive_speed
            // Use this to give the "all clear" later if it never gets set
            blockedF = true;
            // Use this to clear the 'cliff' variable later if this never gets
            // set.
            foundCliff = true;
            blockedSensor[2] = true; // Pretend this is the front sensor, since
                                     // it needs to back up NOW!
            pleaseEscape = true;
          }
        }
      }
      // Clear the global 'cliff' variable if no cliff was seen.
      if (foundCliff == false) {
        cliff = false;
      }
#endif

#ifdef hasFloorObstacleSensors
      floorObstacle = false;
      if (ignoreFloorSensors == false) {
        for (i = 0; i < NUMBER_OF_FLOOR_SENSORS; i++) {
          if (floorArray[i] == 0) {
            // Set the global 'floorO' variable so we can see this in ROS.
            floorO = true;
            safeToProceed =
                false; // Prevent main thread from setting any drive_speed
            // Use this to give the "all clear" later if it never gets set
            blockedF = true;
            // Use this to clear the 'floorO' variable later if this never gets
            // set.
            floorObstacle = true;
            blockedSensor[2] = true; // Pretend this is the front sensor, since
                                     // it needs to back up NOW!
            pleaseEscape = true;
          }
        }
      }
      // Clear the global 'floorO' variable if no floor obstacle was seen.
      if (floorObstacle == false) {
        floorO = false;
      }
#endif

#ifdef hasFrontPingSensors
      // Walk Front PING Sensors to find blocked paths and halt immediately
      for (i = FIRST_FRONT_PING_SENSOR_NUMBER;
           i < HOW_MANY_FRONT_PING_SENSORS + FIRST_FRONT_PING_SENSOR_NUMBER;
           i++) {
        // PING Sensors
        if (pingArray[i] < startSlowDownDistance[i]) {
          if (pingArray[i] <= haltDistance[i] + 1) { // Halt just before.
            safeToProceed = false;
            blockedF = true; // Use this to give the "all clear" later if it
                             // never gets set
            blockedSensor[i] = true; // Keep track of which sensors are blocked
                                     // for intelligent escape sequences.
            // Escape just after, to try make a buffer to avoid back and
            // forthing.
            if (pingArray[i] < haltDistance[i]) {
              pleaseEscape = true;
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
      for (i = FIRST_FRONT_UPPER_SENSOR_NUMBER;
           i < HOW_MANY_FRONT_UPPER_SENSORS + FIRST_FRONT_UPPER_SENSOR_NUMBER;
           i++) {
        // PING Sensors
        if (pingArray[i] < startSlowDownDistance[i]) {
          // Halt just before.
          if (pingArray[i] <= haltDistance[i] + 1) {
            // Prevent main thread from setting any drive_speed
            safeToProceed = false;
            // Use this to give the "all clear" later if it never gets set
            blockedF = true;
            // Keep track of which sensors are blocked for intelligent escape
            // sequences.
            blockedSensor[i] = true;
            if (pingArray[i] < haltDistance[i]) {
              // Escape just after, to try make a buffer to avoid back and
              // forthing.
              pleaseEscape = true;
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
      for (i = FIRST_REAR_PING_SENSOR_NUMBER;
           i < FIRST_REAR_PING_SENSOR_NUMBER + HOW_MANY_REAR_PING_SENSORS;
           i++) {
        if (pingArray[i] < startSlowDownDistance[i]) {
          if (pingArray[i] <= haltDistance[i] + 1) { // Halt just before.
            safeToRecede =
                false;       // Prevent main thread from setting any drive_speed
            blockedR = true; // Use this to give the "all clear" later if it
                             // never gets set
            blockedSensor[i] = true; // Keep track of which sensors are blocked
                                     // for intelligent escape sequences.
            if (pingArray[i] <
                haltDistance[i]) // Escape just after, to try make a buffer to
                                 // avoid back and forthing.
              pleaseEscape = true;
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
      for (i = FIRST_REAR_UPPER_SENSOR_NUMBER;
           i < FIRST_REAR_UPPER_SENSOR_NUMBER + HOW_MANY_REAR_UPPER_SENSORS;
           i++) { // Only use the rear sensors
        // PING Sensors
        if (pingArray[i] < startSlowDownDistance[i]) {
          if (pingArray[i] <= haltDistance[i] + 1) { // Halt just before.
            safeToRecede =
                false;       // Prevent main thread from setting any drive_speed
            blockedR = true; // Use this to give the "all clear" later if it
                             // never gets set
            blockedSensor[i] = true; // Keep track of which sensors are blocked
                                     // for intelligent escape sequences.
            if (pingArray[i] <
                haltDistance[i]) // Escape just after, to try make a buffer to
                                 // avoid back and forthing.
              pleaseEscape = true;
          }
          // For speed restriction:
          if (pingArray[i] < minRDistance) {
            minRDistance = pingArray[i];
            minDistanceSensor = i;
          }
        }
      }
#endif

      if (ignoreIRSensors == false) {
#ifdef hasFrontIRSensors
        // Walk front IR Sensors
        for (i = FIRST_FRONT_IR_SENSOR_NUMBER;
             i < HOW_MANY_FRONT_IR_SENSORS + FIRST_FRONT_IR_SENSOR_NUMBER;
             i++) {
          if (irArray[i] < IRstartSlowDownDistance[i]) {
            if (irArray[i] <= haltDistance[i] + 1) {
              // Prevent main thread from setting any drive_speed
              safeToProceed = false;
              // Use this to give the "all clear" later if it never gets set
              blockedF = true;
              // Keep track of which sensors are blocked for intelligent escape
              // sequences.
              blockedSensor[i] = true;
              if (irArray[i] < haltDistance[i]) {
                pleaseEscape = true;
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
        for (i = FIRST_REAR_IR_SENSOR_NUMBER;
             i < FIRST_REAR_IR_SENSOR_NUMBER + HOW_MANY_REAR_IR_SENSORS; i++) {
#ifdef RENAME_REAR_IR_SENSOR
          int sensorFakeIndex = RENAME_REAR_IR_SENSOR;
#else
          int sensorFakeIndex = i;
#endif
          if (irArray[i] < IRstartSlowDownDistance[i]) {
            if (irArray[i] <= haltDistance[sensorFakeIndex] + 1) {
              safeToRecede =
                  false; // Prevent main thread from setting any drive_speed
              blockedR = true; // Use this to give the "all clear" later if it
                               // never gets set
              blockedSensor[sensorFakeIndex] =
                  1; // Keep track of which sensors are blocked for intelligent
                     // escape sequences.
              if (irArray[i] < haltDistance[sensorFakeIndex])
                pleaseEscape = true;
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
      /* EXPLANATION minDistance won't be set unless a given sensor is closer
       * than its particular startSlowDownDistance value, so we won't be slowing
       * down if sensor 0 is 40, only if it is under 10 */
      if (minDistance < MAX_DISTANCE) {
        // Set based on percentage of range
        newSpeedLimit =
            (minDistance - haltDistance[minDistanceSensor]) *
            (MAXIMUM_SPEED / (MAX_DISTANCE - haltDistance[minDistanceSensor]));
        // Limit maximum and minimum speed.
        if (newSpeedLimit < MINIMUM_SPEED) {
          newSpeedLimit = MINIMUM_SPEED;
        } else if (newSpeedLimit > MAXIMUM_SPEED) {
          newSpeedLimit = MAXIMUM_SPEED;
        }
        // Ramp and limit affect of random hits
        if (newSpeedLimit > abd_speedLimit) {
          if (increaseThrottleRamp == INCREASE_THROTTLE_RATE) {
            abd_speedLimit = abd_speedLimit + 1;
          }
        } else if (newSpeedLimit < abd_speedLimit) {
          if (decreaseThrottleRamp == DECREASE_THROTTLE_RATE) {
            abd_speedLimit = abd_speedLimit - 1;
          }
        }
      } else {
        // Ramp return to full if all obstacles are clear
        if (abd_speedLimit < MAXIMUM_SPEED) {
          if (increaseThrottleRamp == INCREASE_THROTTLE_RATE) // Slow ramping up
            abd_speedLimit = abd_speedLimit + 1;
        }
      }

      // Same for REVERSE Speed Limit
      if (minRDistance < MAX_DISTANCE) {
        // Set based on percentage of range
        newSpeedLimit =
            (minRDistance - haltDistance[minDistanceSensor]) *
            (MAXIMUM_SPEED / (MAX_DISTANCE - haltDistance[minDistanceSensor]));
        // Limit maximum and minimum speed.
        if (newSpeedLimit < MINIMUM_SPEED) {
          newSpeedLimit = MINIMUM_SPEED;
        } else if (newSpeedLimit > MAXIMUM_SPEED) {
          newSpeedLimit = MAXIMUM_SPEED;
        }
        // Ramp and limit affect of random hits
        if (newSpeedLimit > abdR_speedLimit) {
          if (increaseThrottleRamp == INCREASE_THROTTLE_RATE) {
            abdR_speedLimit = abdR_speedLimit + 1;
          }
        } else if (newSpeedLimit < abdR_speedLimit) {
          if (decreaseThrottleRamp == DECREASE_THROTTLE_RATE) {
            abdR_speedLimit = abdR_speedLimit - 1;
          }
        }
      } else {
        // Ramp return to full if all obstacles are clear
        if (abdR_speedLimit < MAXIMUM_SPEED) {
          if (increaseThrottleRamp == INCREASE_THROTTLE_RATE) // Slow ramping up
            abdR_speedLimit = abdR_speedLimit + 1;
        }
      }

      // Clear forward and backward individually now.
      if (!blockedF) {
        safeToProceed = true;
      }
      if (!blockedR) {
        safeToRecede = true;
      }

      // If NO sensors are blocked, give the all clear!
      if (blockedF == false && blockedR == false) {
        Escaping = false; // Have fun!
      } else {
        if (pleaseEscape == true && pluggedIn == false) {
          // If it is plugged in, don't escape!
          Escaping =
              true; // This will stop main thread from driving the motors.
          /* At this point we are blocked, so it is OK to take over control
             of the robot (safeToProceed == false, so the main thread won't do
             anything), and it is safe to do work ignoring the need to slow down
             or stop because we know our position pretty well. HOWEVER, you will
             have to RECHECK distances yourself if you are going to move in this
             program location.
             */
          if (safeToRecede == true) {
// The order here determines priority.
#ifdef FRONT_CENTER_SENSOR
            if (blockedSensor[FRONT_CENTER_SENSOR]) {
              setEscapeSpeeds(-MINIMUM_SPEED, -MINIMUM_SPEED);
#ifdef FRONT_3D_MOUNTED_SENSOR
            } else if (blockedSensor[FRONT_3D_MOUNTED_SENSOR]) {
              setEscapeSpeeds(-MINIMUM_SPEED, -MINIMUM_SPEED);
#endif
#ifdef FRONT_UPPER_DECK_CENTER_SENSOR
            } else if (blockedSensor[FRONT_UPPER_DECK_CENTER_SENSOR]) {
              setEscapeSpeeds(-MINIMUM_SPEED, -MINIMUM_SPEED);
#endif
#ifdef FRONT_NEAR_LEFT_SENSOR
            } else if (blockedSensor[FRONT_NEAR_LEFT_SENSOR]) {
              setEscapeSpeeds(-MINIMUM_SPEED,
                              -(MINIMUM_SPEED * 2)); // Curve out to the right
#endif
#ifdef FRONT_UPPER_DECK_NEAR_LEFT_SENSOR
            } else if (blockedSensor[FRONT_UPPER_DECK_NEAR_LEFT_SENSOR]) {
              setEscapeSpeeds(-MINIMUM_SPEED,
                              -(MINIMUM_SPEED * 2)); // Curve out to the right
#endif
#ifdef FRONT_NEAR_RIGHT_SENSOR
            } else if (blockedSensor[FRONT_NEAR_RIGHT_SENSOR]) {
              setEscapeSpeeds(-(MINIMUM_SPEED * 2),
                              -MINIMUM_SPEED); // Curve out to the left
#endif
#ifdef FRONT_UPPER_DECK_NEAR_RIGHT_SENSOR
            } else if (blockedSensor[FRONT_UPPER_DECK_NEAR_RIGHT_SENSOR]) {
              setEscapeSpeeds(-(MINIMUM_SPEED * 2),
                              -MINIMUM_SPEED); // Curve out to the left
#endif
#ifdef FRONT_FAR_LEFT_SENSOR
            } else if (blockedSensor[FRONT_FAR_LEFT_SENSOR]) {
              setEscapeSpeeds(0,
                              -MINIMUM_SPEED); // Turn out to the right slowly
#endif
#ifdef FRONT_FAR_RIGHT_SENSOR
            } else if (blockedSensor[FRONT_FAR_RIGHT_SENSOR]) {
              setEscapeSpeeds(-MINIMUM_SPEED, 0); // Turn out to the left slowly
#endif
            }
#endif
          } else if (safeToProceed ==
                     true) { // Escaping for rear sensors, these will move more
                             // generically forward.
#ifdef REAR_CENTER_SENSOR
            if (blockedSensor[REAR_CENTER_SENSOR]) {
              setEscapeSpeeds(MINIMUM_SPEED, MINIMUM_SPEED);
#ifdef REAR_3D_MOUNTED_SENSOR
            } else if (blockedSensor[REAR_3D_MOUNTED_SENSOR]) {
              setEscapeSpeeds(MINIMUM_SPEED, MINIMUM_SPEED);
#endif
#ifdef REAR_UPPER_DECK_SENSOR
            } else if (blockedSensor[REAR_UPPER_DECK_SENSOR]) {
              setEscapeSpeeds(MINIMUM_SPEED, MINIMUM_SPEED);
#endif
#ifdef REAR_NEAR_RIGHT_SENSOR
            } else if (blockedSensor[REAR_NEAR_RIGHT_SENSOR]) {
              setEscapeSpeeds(MINIMUM_SPEED, MINIMUM_SPEED * 2);
#endif
#ifdef REAR_NEAR_LEFT_SENSOR
            } else if (blockedSensor[REAR_NEAR_LEFT_SENSOR]) {
              setEscapeSpeeds(MINIMUM_SPEED * 2, MINIMUM_SPEED);
#endif
#ifdef REAR_FAR_RIGHT_SENSOR
            } else if (blockedSensor[REAR_FAR_RIGHT_SENSOR]) {
              setEscapeSpeeds(MINIMUM_SPEED, 0);
#endif
#ifdef REAR_FAR_LEFT_SENSOR
            } else if (blockedSensor[REAR_FAR_LEFT_SENSOR]) {
              setEscapeSpeeds(0, MINIMUM_SPEED);
#endif
            }
#endif
          } else { // We are trapped!!
            // Turns out we cannot escape, so turn off "Escaping",
            // and now drive control should refuse to move forward or back,
            // due to safeToRecede & safeToProceed both == false,
            // but it should be willing to rotate in place,
            // which is a normal function of both arlobot_explore
            // and the navigation stack's "clearing" function.
            Escaping = false;
          }
        } else { // This is the "halt" but don't "escape" action for that middle
                 // ground.
          if (Escaping == true) {  // If it WAS Escaping, stop it now.
            setEscapeSpeeds(0, 0); // return to stopped before giving control
                                   // back to main thread
          }
          Escaping = false; // Blocked, but not close enough to Escape yet
        }
      }

      increaseThrottleRamp = increaseThrottleRamp + 1;
      if (increaseThrottleRamp > INCREASE_THROTTLE_RATE)
        increaseThrottleRamp = 0;
      decreaseThrottleRamp = decreaseThrottleRamp + 1;
      if (decreaseThrottleRamp > DECREASE_THROTTLE_RATE)
        decreaseThrottleRamp = 0;

    } else {
      /* All limits and blocks must be cleared if we are going to ignore
      proximity. Otherwise we get stuck! */
      Escaping = false;
      safeToProceed = true;
      safeToRecede = true;
      cliff = false;
      floorO = false;
      abd_speedLimit = MAXIMUM_SPEED;
      abdR_speedLimit = MAXIMUM_SPEED;
    }
    pause(1); // Just throttles this cog a little.
  }
}

#endif /* SAFETY_OVERRIDE_COG_H */
