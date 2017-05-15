#include "per_robot_settings_for_propeller_c_code.h"
// TESTING:

/*
 * Use this to test the Arlobot performance characteristics for setting values
 * and parameters for ROS
 */

/* If you want to experiment with the maximum acceleration on the DHB-10
 * edit DHB10_ACC in per_robot_settings_for_propeller_c_code.h
 */

/* Adjust the
 * COMMANDED_VELOCITY
 * and
 * COMMANDED_ANGULAR_VELOCITY
 *
 * These are the components of the "Twist" message sent by ROS to the robot:
 *
            v = twist_command.linear.x  # m/s
            omega = twist_command.angular.z  # rad/s
 */

#define COMMANDED_VELOCITY 0.0 // m/s
#define COMMANDED_ANGULAR_VELOCITY -3.6 // rad/s

// Do it forever? (Not really, it will stop at the MAXIMUM_LOOPS count)
#define FOREVER 0; // 0 for not, in which case it stops when it reaches the stated velocity.

// Prevent runaway, especially useful for max/min speed testing.
// Start with a safe value like 50, and raise it if it hits this before reaching your requested speed.
#define MAXIMUM_LOOPS 50;

// Experiment to determine:
// Maximum speed in linear and angular.
/* To test for max speed, increase slowly until you get to a point
where the robot never attains the speed and then back down
to a number it reaches consistently.
Note that if you are on the edge, it will reach the speed,
but with different times every run because it is struggling,
or because the motors are not consistent.
Bump it down a couple of 10th's of a m/s.
I found the max to be 0.82,
but I'm using 0.80 as a maximum to keep it consistent.

NOTE: Max speed with the wheels in the air is NOT
going to work with the wheels on the ground!
*/
// Acceleration in linear and angular.
/* Use the max speed you obtained above,
and look at the time required to reach it.
Do this at least 10 times, and either use an average,
or just use the number that comes up most often.

NOTE: It may make more sense to test the acceleration
with the robot actually moving itself, not blocked up.
You can get your max speed blocked up, but then test
it again on a surface, and test acceleration there too.
Acceleration is both slowed by having friction,
but also the wheels hunt a bit to find full speed when they
have no friction, even passing the requested speed.
*/
// Minimum speed in linear and angular.
/* This is a bit more of a judgement call.
Basically find a speed where it will operate continuously
without being jerky.
It is pretty obvious when it is surging, vs. when it cna
actually maintain the speed.
I suggest you COMMENT OUT the line toward the bottom that
stops the robot when it reaches speed so that it will just run at the
slow speed forever, and then watch to see if it is jerky.
It shouldn't run away from you very far at this rate. :)
*/

/*
    The acceleration will vary by about 200ms, because
    the cycle is 100ms, so it can fall across that.
    If you find it varying a lot though, you probably
    have the max speed too high and it is hunting or
    struggling.
    Bump it down a notch so it can be consistent.
    I find I get 7 or 8 runs with identical acceleration
    times.
*/

/* NOTICE how the angular is in rad/s
In theory you can just set the result below in m/s,
but ROS will send it in rad/s and the tunable
parameters for rotation are in rad/s, not m/s
because m/s requires knowing the geometry of the roobt,
as the formula below illustrates. */

#include "fdserial.h"
#include "simpletools.h"

/*
Full details on how to use the DHB-10 Motor Controller on the Parallax Arlo
Robot platform with a Propeller Activity Board can be found here:
http://learn.parallax.com/tutorials/robot/arlo/arlo-activity-board-brain
I highly suggets you work through the instructions there and run the example
programs and tests before using this code.
*/
#include "arlodrive.h"

fdserial *term;

const char delimiter[2] = ","; // Delimiter character for incoming messages from the ROS Python script

// For DHB-10 Interaction See drive_speed.c for example code
char dhb10_reply[DHB10_LEN];

int main() {

    // Robot description: We will get this from ROS so that it is easier to
    // tweak between runs without reloading the Propeller EEPROM.
    // http://learn.parallax.com/activitybot/calculating-angles-rotation
    // See
    // ~/catkin_ws/src/ArloBot/src/arlobot/arlobot_bringup/param/arlobot.yaml to
    // set or change this value
    double distancePerCount = 0.0, trackWidth = 0.0;

    // For DHB-10 Interaction See drive_speed.c for example code
    // NOTE: Because this function has a loop, ALL interaction with the DHB-10
    // is done in this main loop. Any other cog/function that needs to affect
    // the robot's motors will set variables that are read in this function.
    char s[32]; // Hold strings converted for sending to DHB-10
    memset(s, 0, 32);
    char *reply = dhb10_reply;

    // Halt motors in case they are moving and reset all stats.
    dhb10_com("GOSPD 0 0\r");
    pause(dhb10OverloadPause);
    dhb10_com("RST\r");
    pause(dhb10OverloadPause);
    sprint(s, "ACC %d\r", DHB10_ACC);
    dhb10_com(s);
    pause(dhb10OverloadPause);

    // For Odometry
    int ticksLeft, ticksRight, ticksLeftOld, ticksRightOld;
    double Heading = 0.0, X = 0.0, Y = 0.0, deltaDistance, deltaX, deltaY, V = 0.0, Omega;
    int speedLeft, speedRight, throttleStatus = 0, heading, deltaTicksLeft, deltaTicksRight, leftCalcSpeed,
            rightCalcSpeed;
    double leftMotorPower = 4.69;
    double rightMotorPower = 4.69;
    int wasEscaping = 0;

    simpleterm_close();                      // Close simplex serial terminal
    term = fdserial_open(31, 30, 0, 115200); // Open Full Duplex serial connection

    // For Debugging without ROS:
    // See
    // ~/catkin_ws/src/ArloBot/src/arlobot/arlobot_bringup/param/arlobot.yaml
    // for most up to date values
    trackWidth = 0.403000; // from measurement and then testing
    distancePerCount = 0.00338;
    // http://forums.parallax.com/showthread.php/154274-The-quot-Artist-quot-robot?p=1271544&viewfull=1#post1271544

    // Declaring variables outside of loop
    // This may or may not improve performance
    // Some of these we want to hold and use later too
    // A Buffer long enough to hold the longest line ROS may send.
    const int bufferLength = 35; // A Buffer long enough to hold the longest line ROS may send.
    char buf[bufferLength];
    int count = 0, i = 0;

    // To hold received commands
    double CommandedVelocity = COMMANDED_VELOCITY;
    double CommandedAngularVelocity = COMMANDED_ANGULAR_VELOCITY;
    double angularVelocityOffset, expectedLeftSpeed, expectedRightSpeed;
    int newLeftSpeed, newRightSpeed;

    // This is just a test of the system and this code.
    // It should always be 1000ms
    dprint(term, "Testing:\npause(1000) takes ... ");
    int millisecond = CLKFREQ / 1000;
    int time1 = -CNT; // Get current time as a negative
    pause(1000);
    time1 += CNT; // Now subtract it to see how long this takes
    int difference1 = time1 / millisecond;
    dprint(term, "%d milliseconds.\n", difference1);

    // Just tell us how long it takes to read the time
    // Again, just a test of the system and this code. Should be 0ms
    int time2 = -CNT;
    time2 += CNT; // Now subtract it to see how long this takes
    int difference2 = time2 / millisecond;
    dprint(term, "Checking the time takes %d milliseconds.\n", difference2);

    dprint(term, "Running:\n");

    int started = 0;
    int reachedMaxSpeed = 0;
    int stopping = 0;
    int completed = 0;
    int printDetails = 0;
    int difference, time;
    int forever = FOREVER;
    int maximumLoops = MAXIMUM_LOOPS;

    angularVelocityOffset = CommandedAngularVelocity * (trackWidth * 0.5);

    expectedLeftSpeed = CommandedVelocity - angularVelocityOffset;
    expectedRightSpeed = CommandedVelocity + angularVelocityOffset;

    expectedLeftSpeed = expectedLeftSpeed / distancePerCount;
    expectedRightSpeed = expectedRightSpeed / distancePerCount;

    newLeftSpeed = (int) expectedLeftSpeed;
    newRightSpeed = (int) expectedRightSpeed;

    if (newLeftSpeed % 2) {
        newLeftSpeed >= 0 ? newLeftSpeed++ : newLeftSpeed--;
    }
    if (newRightSpeed % 2) {
        newRightSpeed >= 0 ? newRightSpeed++ : newRightSpeed--;
    }

    int dt = CLKFREQ / 10;
    // Operates once per every 100 ms (Just like the ROS Interface)
    // https://lamestation.atlassian.net/wiki/display/SPIN/CLKFREQ
    int t = CNT;
    while (completed == 0) {
        if (CNT - t > dt) {
            t += dt;
            count++;

            if (V == 0.0 && started == 0) {
                started = 1;
                dprint(term, "Target:\n%d\t%d\t%.3f\nReal time:\n", newLeftSpeed, newRightSpeed, V);
                time = -CNT; // Get current time as a negative
                sprint(s, "GOSPD %d %d\r", newLeftSpeed, newRightSpeed);
                dhb10_com(s);
            }

            ticksLeftOld = ticksLeft;
            ticksRightOld = ticksRight;

            pause(dhb10OverloadPause);
            reply = dhb10_com("DIST\r");
            if (*reply == '\r') {
                ticksLeft = 0;
                ticksRight = 0;
            } else {
                sscan(reply, "%d%d", &ticksLeft, &ticksRight);
            }
            // dprint(term, "d\tDIST\t%d\t%d\n", ticksLeft, ticksRight);  // For Debugging

            pause(dhb10OverloadPause);
            reply = dhb10_com("SPD\r");
            if (*reply == '\r') {
                speedLeft = 0;
                speedRight = 0;
            } else {
                sscan(reply, "%d%d", &speedLeft, &speedRight);
            }
            // dprint(term, "d\tSPEED\t%d\t%d\n", speedLeft, speedRight);  // For Debugging

            pause(dhb10OverloadPause);
            reply = dhb10_com("HEAD\r");
            if (*reply == '\r') {
                heading = 0;
            } else {
                heading = atoi(reply);
            }
            // dprint(term, "d\tHEADING\t%d\n", heading);  // For Debugging
            // The heading is apparently reversed in relation to what ROS expects,
            // hence the "-heading"
            Heading = -heading * PI / 180.0; // Convert to Radians

            deltaTicksLeft = ticksLeft - ticksLeftOld;
            deltaTicksRight = ticksRight - ticksRightOld;
            deltaDistance = 0.5f * (double) (deltaTicksLeft + deltaTicksRight) * distancePerCount;
            deltaX = deltaDistance * cos(Heading);
            deltaY = deltaDistance * sin(Heading);

            X += deltaX;
            Y += deltaY;

            // http://webdelcire.com/wordpress/archives/527
            V = ((speedRight * distancePerCount) + (speedLeft * distancePerCount)) / 2;
            Omega = ((speedRight * distancePerCount) - (speedLeft * distancePerCount)) / trackWidth;

            dprint(term, "%d\t%d\t%.3f\n", speedLeft, speedRight, V);

            if (reachedMaxSpeed == 0 && abs(speedLeft) >= abs(newLeftSpeed) && abs(speedRight) >= abs(newRightSpeed)) {
                // The speed varies, so if you don't use >=, it can go a long time hunting.
                time += CNT; // Now subtract it to see how long this takes
                difference = time / millisecond;
                if (CommandedVelocity != 0.0) {
                    dprint(term, "It took %d millisecond to reach %.3f m/s.\n", difference, CommandedVelocity);
                }
                if (CommandedAngularVelocity != 0.0) {
                    dprint(term, "It took %d millisecond to reach %.3f rad/s.\n", difference, CommandedAngularVelocity);
                }
                //            printDetails = 1;
                time = -CNT; // Get current time as a negative
                reachedMaxSpeed = 1;
                if (forever == 0) {
                    stopping = 1;
                    dhb10_com("GOSPD 0 0\r");
                    pause(dhb10OverloadPause);
                    sprint(s, "ACC %d\r", DHB10_ACC);
                    dhb10_com(s);
                }
            } else if (stopping == 1 && speedLeft == 0 && speedRight == 0) {
                time += CNT; // Now subtract it to see how long this takes
                difference = time / millisecond;
                dprint(term, "It took %d millisecond to come to a complete stop.\n", difference);
                //            printDetails = 1;
                stopping = 0;
                completed = 1;
            }

            if (count == maximumLoops && stopping == 0 && completed == 0) {
                dprint(term, "Maximum loop count hit.");
                dhb10_com("GOSPD 0 0\r");
                completed = 1;
            }
        }
    }
}
