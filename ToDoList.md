##TODO List:##

Run this to see what it should see:
rosrun tf tf_echo /map /base_link

Run this to see what really happens:
cd catkin_ws/src/ArloBot/node
node getCurrentPosition.js

Why doesn't it get the output of the rosrun command?

Use these to put in some delay and idle checkers:
var bootTime = new Date() // Time the node script was initialized
var startROSTime = new Date() // Time that ROS start was completed.
var mapLoadTime = new Date() // Time that map load was complete

TEST:
After a set time ON,  if no QRcode, turn on light for a set time.
if (!qrCodeFound)
    turnonLight, setTimeout(funciton() {
    if (!lightOnUserRequest) {
        turnofflight;
    }
    })

Test timeout before it asks where it is or if I can unplug it.
    Remember to include the turn on light time.

If you try to plug him in when he is already charged, he unplugs himself again,
This is actually fun, but I want:
1. an override
2. him to comment on it.

0. Test and move PING, RESET and STAYCLOSE from node to public.

0. If they run setup, but have their OWN local changes, we need to warn them,
that there are differences, instead of just saying, "up to date."

0. Test: zbarcam is pretty CPU intensive, so it shouldn't run if the robot isn't idle.

0. When idle proceed to random locations from waypoint list.

0. Create speech tree and give text for locations.

0. Go to certain waypoints based on input such as motion cameras and door alarms.

0. Tweak cost_scaling_factor and inflation_radius of global and local planner
in order to produce smooth path planning into all expected areas without running
into corners or doorframes.
     * Doorways make it slow down too much,
this is very obvious during autonomous navigation,
but it is easily tested with keyboard teleop.
So fix the speed regulation through doors,
probably by adjusting how/when/if the angled sensors affect speed limits.
     * Also, is there a way to have the global "expansion variable" be wide enough for it to
give wide berths to corners,
but not cause it to get stuck trying to get through a door?
0.5 is great for corners, but doors are often blocked,
0.25 is great for doors, but it cuts off the corners do close it runs over things
Maybe ask on ROS answers?

0. Add function for robot to spin in a slow circle if it is stuck, based on
failing navigation attempts from both WayPoints and Auto Explore (harder).

0. Question: Why doesn't it do a rotate in place when it gets stuck and cannot
create a plan? I thought that was built in.

0. Consider how to handle timeout on waypoint requests.

0. How to tell when it is "idle" for idle behaviors?
     * /cmd_vel_mux/active - GOOD
        * This will say "data: idle" if nothing is driving the robot.
     * /mobile_base/commands/velocity - GOOD
        * Should be BLANK/EMPTY unless the robot is moving.
     * /move_base/status - Good
        * This is continuous, so look for what you want and see if it repeats X times.
        * Examples of what is good:
            * text: Goal reached
            * Not sure what this does before first goal?
     * /odom - GOOD
        * Everything except for seq, secs and nsecs should repeat, so grab X instances and compare for idleness
     * /serial - GOOD
        * Continuous, and it should tell you if the motors are still.
     * Set "switch" to idle on web panel, so it can be turned off or on manually for testing?

0. Keyboard teleop is very smooth now!
Can the joystick and web op be run through a smoother too?

0. Touch screen teleop?

0. Set up some way to emulate robot for testing?
http://cs.smith.edu/dftwiki/index.php/PySerial_Simulator

0. Is explore pause working, seems inconsistent.

0c. speech behavior tree.

0d. Upper PING sensors need to be integrated into fake laser data.
    Not sure if this is going to be the SAME, or if we filter the data?
        i.e. Do we need to only pass in signals below a certain distance?
        or above a certain distance?
    or overlay it, or what?
        shortest wins?
        or smarter?
    This will be a pattern for future adds too.
    Remember, the planner is 2D essentially, so no need for multiple rows.

0e. Upper front sensor may need to be adjusted so it doesn't see the front of the robot?

0f. Is the obstacle avoidance within the propeller code stopping the robot too fast?
can it haver a way to slow it to a rapid stop instead of dropping it from 100 to 0 instantly, even if the obstacle wasn't seen in time to use the speed limiting?

5. Create a procedure to get and keep a good map of each floor of my house.
    * It will have to be updated periodically
    * But I need to be able to just pull it up instead of doing an "Explore" every time and then fussing with it for an hour.

6. Move speech from babel-fish python ROS node to a behavior3JS based node module controlled by the node program.

8. Interface with SMS text messages so that text messages an tell the robot
to:
load a map,
go to a way-point.

9. Integrate the alarm system with the same, so that open doors will send the robot somewhere.

10. Add speech for all of the statuses.

11. Add "idle" behavior and speech to the robot.
    * Include wandering and looking for people (see 16.)

13. Can a QR code be used to reset the robot's position to a known one?

14. Follow me. Does this have to be an object?

15. Conversational speech.

16. Face detection and recognition with either camera while navigating the map, presumably with the XV11 (so that the ASUS xTion is free).
    * Could the robot wander the house to find a person?
    * Could it find a specific person, either by facial recognition or by conversation to figure out who someone is?

17. NEOPIXELS!

18. Movable platform for Kinect sensor with Dynamixels!

19. AN ARM?!

20. Make it taller?!

21. Jasper? https://jasperproject.github.io/

22. Check for Turtlebot improvements that might affect me.
    * Might this be an issue? http://blog.yujinrobot.com/2015/03/the-quest-for-accurate-navigation-i-my.html

## Done! ##

0. Can it not ask "where am I" if the QR code is there?
    Seems maybe it is just asking too soon or something?

0. Need to add "ignore floor sensors" to web intarface.

0. Still doesn't move if there are files in doors even if they do not correlate to a map.
