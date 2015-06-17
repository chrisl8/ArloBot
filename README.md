Metatron Package for Arlobot
===========================

![Alt text](/screenshots/robotControlPanel.png?raw=true "Control Panel")

This is a supplementary package for the Arlobot package.
It provides basic "personality" and autonomous operation functions
to my ROS robot.

This package will be much less generic, depending more highly
on my specific robot design.

##Setup##
Do not set up this package alone. It only works with the Arlobot Package.
Go to the ArloBot package https://github.com/chrisl8/ArloBot and follow the setup instructions there, and this package will also be installed automatically.

##Convenience Scripts##
Look in the scripts folder for a set of handy scripts for starting up and shutting down various aspects of Arlobot.

##Local and Remote Web Interface##
In the scripts folder run behavior.sh to start up a Node.js based behavior system and web interface. You can connect to the interface by going to http://localhost:8080/localmenu.html on the robot's computer, or replace "localhost" with the robot computer's IP or host name for remote operation.
