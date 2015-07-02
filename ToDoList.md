##TODO List:##

0. Play more Minecraft!

1. Move roslibjs functions to their own module

2. Create a way for any module to request things from the roslibjs module,
i.e. webserver needs to be able to change parameters

    a. Implement and test cliff, IR and Proximity ignore switches.
    b. Test Explore Pause function (seems flaky)

3. Rearrange web pages so that index.html is the localmenu.html and the "ArloWeb" page is a separate html page (possibly called "remote control"?)
    Be sure to update any scripts or links that point to this stuff, such as in the XWindows startup icon script

    a. Make sure "kiosk" page still works now!

4. Create a script in MetaTron git repo root to start "behavior.sh" in place of the one in the scripts folder, and delete that one,
again, update scripts.

5. Create a procedure to get and keep a good map of each floor of my house.
    * It will have to be updated periodically
    * But I need to be able to just pull it up instead of doing an "Explore" ever time and then fussing with it for an hour.

5. Move propeller C code customizations into ~/.arlobot somehow so that updates to the git repo do not have to affect changes to personal settings.

6. Move speech from babel-fish python ROS node to a behavior3JS based node module controlled by the node program.

7. Create a system to set "waypoints" in the map from the web interface,
and to list them in the web interface,
and to send the robot to them.

8. Interface these things with SMS text messages so that text messages an tell the robot
to:
load a map,
go to a way-point.

9. Integrate the alarm system with the same, so that open doors will send the robot somewhere.

10. Add speech for all of the statuses.

11. Add "idle" behavior and speech to the robot.
    * Include wandering and looking for people (see 16.)

12. Set up robot to unplug itself in the basement (like upstairs).

13. Set up "QR Code" to put on wall in place so robot knows at power on where it is (basement or upstairs) and other locatability (maybe it can go to a spot and verify it finds the right QR code?)

14. Follow me. Does this have to be an object?

15. Conversational speech.

16. Face detection and recognition with either camera while navigating the map, presumably with the XV11 (so that the ASUS xTion is free).
    * Could the robot wander the house to find a person?
    * Could it find a specific person, either by facial recognition or by conversation to figure out who someone is?

17. NEOPIXELS!

18. Movable platform for Kinect sensor with Dynamixels!

19. AN ARM?!

20. Make it taller?!
