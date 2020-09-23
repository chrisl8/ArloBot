## TODO List

 - Speak when things are done like starting ROS, making maps, etc.

 - Fix this error coming from TEB due to something in my config:
    [ WARN] [1597326961.776132466]: Control loop missed its desired rate of 20.0000Hz... the loop actually took 0.0513 seconds
    - Might have to look into http://wiki.ros.org/teb_local_planner/Tutorials/Costmap%20conversion

 - Test remaining "goto" functions with Python 3 and Slam Toolbox

 - Watch for new ros-noetic-tf2 to release and ensure I'm getting it, and the ros-testing one I do not have anymore.

 - Add pip updater to setup script.
### Check for and update ONLY user packages (OS Level are Ubuntu's problem . . . until they aren't)
    pip3 install pip-review

    pip-review --user
    #pip-review --user --auto

 - Have the robot say "pardon me" when escaping starts
 
 - Set every button push to have a function
 
 - Get all Sensors to work on Threeflower
 
 - Get Slam Toolbox to come up on Threeflower.
 
 - Mycroft
   - Set up a skill to go to load map, unplug, go to waypoints, etc.
   - Voice command to go to locations.
 
 - Improve web interface look and feel.
   - The old LCARS look was fun, can I find something like that to easily skin on?
 
 - Test everything and dump stuff that doesn't work anymore.
   - See FullTestRoutine.MD
   
 - Walk through entire Readme and ensure all instructions are correct/relevant.
 
 - Make a new Youtube video of robot functions.

 - Create a system on the web interface to create automatic mapping of an existing area
    - Start map
    - Select goals in RVIZ
    - System should save them and put them up on the website
    - You can see them in order.
    - Then you can save the "list of goals"
    - Later you can select to make map, and it will start a map making session, unplug, go to those goals in order, then save the map, go "home", then load the map use it.
        - This way you can remap a known area automatically.
    - Early start might be to just set up to monitor goals and list them out on the site as they are set.

 - Threeflower testing
  - robot.launch file:
    - Set up Scanse to be able to be Scan topic.
    - Set RPLidar section of up like XV11 to use /rplidar by default, but /scan if "selected" which is default if nothing is passed in.
    - Test to ensure that if NO ENVIRONMENT VARIABLES OR ROS PARAMS are set, that it uses RPLidar by default.

 - arlobot_goto.py
    - See TODO's in it: Need to either clean out old "active input" from previous mux package, or update twist_mux to give us the active topic and use it.