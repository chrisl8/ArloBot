## TODO List

 - Watch for new ros-noetic-tf2 to release and ensure I'm getting it and the ros-testing one I do not have anymore.

 - Add pip updater to setup script.
### Check for and update ONLY user packages (OS Level are Ubuntu's problem . . . until they aren't)
    pip3 install pip-review

    pip-review --user
    #pip-review --user --auto

 - Have the robot say "pardon me" when escaping starts
 
 - Set every button push to have a function
 
 - Voice command to go to locations.
 
 - Get all Sensors to work on Threeflower
 
 - Get Slam Toolbox to come up on Threeflower.
 
 - Mycroft
   - Speak when things are done like starting ROS, making maps, etc.
   - Set up a skill to go to load map, unplug, go to waypoints, etc.
 
 - Improve web interface look and feel.
   - The old LCARS look was fun, can I find something like that to easily skin on?
 
 - Test everything and dump stuff that doesn't work anymore.
   - See FullTestRoutine.MD
   
 - Walk through entire Readme and ensure all instructions are correct/relevant.
 
 - Make a new Youtube video of robot functions.

 - Create a system in the web to create automatic mapping of an existing area
    - Start map
    - Select goals in RVIZ
    - System should save them and put them in the web site
    - You can see them in order.
    - Then you can save the "list of goals"
    - Later you can select to make map and it will start a map making session, unplug, go to those goals in order, then save the map, go "home", then load the map use it.
        - This way you can remap a known area automatically.
    - Early start might be to just set up to monitor goals and list them out on the site as they are set.

 - Set up some logging "options".
    - Debugging
    - Console
    - Behavior loops
        - Probably want check boxes so you can have multiple active at once
