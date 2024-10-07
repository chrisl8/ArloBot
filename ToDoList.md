## ABANDONED TODO List 

**I am no loner planning to do any of this!** Sorry if you were waiting on me, but I have moved on to other things.

 - [x] Make a new Youtube video of robot functions.
 
 - Add some response to failed arrival:
    - Text message.
    - Fun TTS and/or audio clip.
    - Track and if repeated, affect the random destination picker.

 - Speak when things are done like starting ROS, making maps, etc.
    - Asked to unplug
    - Uplugged

 - Fix this error coming from TEB due to something in my config:
    [ WARN] [1597326961.776132466]: Control loop missed its desired rate of 20.0000Hz... the loop actually took 0.0513 seconds
    - Might have to look into http://wiki.ros.org/teb_local_planner/Tutorials/Costmap%20conversion

 - Set every button push to have a function
 
 - Get all Sensors to work on Threeflower
 
 - Get Slam Toolbox to come up on Threeflower.

 - Test everything and dump stuff that doesn't work anymore.
   - See FullTestRoutine.MD
   
 - Walk through entire Readme and ensure all instructions are correct/relevant.
 
 - Create a system on the web interface to create automatic mapping of an existing area
    - Start map
    - Select goals in RVIZ
    - System should save them and put them up on the website
        - Subscribe to /move_base_simple/goal to see goals.
        
    - You can see them in order.
    - Then you can save the "list of goals"
    - Later you can select to make map, and it will start a map making session, unplug, go to those goals in order, then save the map, go "home", then load the map use it.
        - This way you can remap a known area automatically.
    - Early start might be to just set up to monitor goals and list them out on the site as they are set.
    
    NOTES:
    Options:
    /move_base/cancel
    /move_base/feedback
    /move_base/goal
    /move_base/parameter_descriptions
    /move_base/parameter_updates
    /move_base/result
    /move_base/status
    /move_base_simple/goal
    
    
    What they do:
    /move_base/goal - announces new goals when set
    /move_base_simple/goal - Same, just less data
    /move_base/current_goal - Seems to be the same, but THIS IS WHAT RVIZ IS SHOWING!
    /move_base/result - Will only tell you when it is done doing something.
    
    It seems like I want to watch:
    /move_base/current_goal - To see when a goal is set,
    and
    /move_base/result - To see when one is reached (or abandoned).
    
    I can infer that whatever came in via /move_base/current_goal last is what was reached or not.
    
    I *could* also subscribe to /move_base/status if I don't want to maintain state of any kind,
    but that seems unnecesary.
    
    See rosInterface.js for subscribing to topics and acting on those subscriptions by pumping data into webmodel/robotmodel and/or TTS, etc.
    
    TODO: Unsure how to "return to base". It isn't actually just all zereos is it?
        Maybe I need to "grab" the location upon initial startup (before we unplug).

 - Threeflower testing
  - robot.launch file:
    - Set up Scanse to be able to be Scan topic.
    - Set RPLidar section of up like XV11 to use /rplidar by default, but /scan if "selected" which is default if nothing is passed in.
    - Test to ensure that if NO ENVIRONMENT VARIABLES OR ROS PARAMS are set, that it uses RPLidar by default.

 - arlobot_goto.py
    - See TODO's in it: Need to either clean out old "active input" from previous mux package, or update twist_mux to give us the active topic and use it.
