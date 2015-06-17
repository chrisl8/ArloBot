Metatron Package for Arlobot
===========================

![Alt text](/screenshots/robotControlPanel.png?raw=true "Control Panel")

This is a supplementary package for the Arlobot package.
It provides basic "personality" and autonomous operation functions
to my ROS robot.

This package will be much less generic, depending more highly
on my specific robot design.

##Setup##
Run the setup.sh script from inside of the scripts folder to set up
everything Metatron and Arlobot need to run.

##Convenience Scripts##
Look in the scripts folder for a set of handy scripts for starting up and shutting down various aspects of Arlobot.

##Local and Remote Web Interface##
In the scripts folder run behavior.sh to start up a Node.js based behavior system and web interface. You can connect to the interface by going to http://localhost:8080/localmenu.html on the robot's computer, or replace "localhost" with the robot computer's IP or host name for remote operation.
