##TODO List:##

0. Test RVIZ shortcut.

0. Unplug itself if it knows it is on an "unplugable" map,
based on setting and QR code.

0. Test and move PING, RESET and STAYCLOSE from node to public.

0. Robot should localize itself (2D pose estimate) based on a certain "waypoint" in the list,
if it exists. (i.e. It should locate itself in the basement as soon as it is loaded.)

0. Test: zbarcam is pretty CPU intensive, so it shouldn't run if the robot isn't idle.

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

0. Need to be able to turn floor sensors on/off from web page.

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

12. Set up robot to unplug itself in the basement (like upstairs).

12. Send "initial" waypoint robot's "initial position" once map is loaded, if such a waypoint exists, so that the robot can automatically be started at the right place on the map.

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
