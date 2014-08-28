Propeller C Code for ArloBot
============================

This is the code to build in SimpleIDE and download to your ArloBot's Activity Board EEPROM in order to communicate with ROS over the USB serial interface.

You will need to have the Parallax ArloBot Libraries in your SimpleIDE workspace in order for this to build. Those libraries are included in this github repository, or you can get them here:
http://forums.parallax.com/showthread.php/152636-Run-ActivityBot-C-Code-in-the-Arlo!?p=1230592&amp;amp;posted=1#post1230592

Please note that this code assumes that your Activity Board is set up in a particular way, with specific sensors on specific pins, etc. If the layout is not clear from the code just let me know and I will write up some instructions and take some pictures to explain how to make your Activity Board set up like mine. Everything in the code is hard coded. You can easily change pin locations and delete code for items that you don't have, however if you would like me to change the code to make it easier to adjust for various configurations just let me know and I will add some code to make it easier to flip things like the gyro on or off based on your configuration.
