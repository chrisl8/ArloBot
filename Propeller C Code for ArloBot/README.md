Propeller C Code for ArloBot
============================

This is the code to build in SimpleIDE and download to your ArloBot's Activity Board EEPROM in order to communicate with ROS over the USB serial interface.

## Two Versions Now ##
Note, I now have a second Propeller based Quickstart board on my Arlo. All PING and Infrared sensors are on this board. This means I have two versions of the code here.
* ROS Interface for ArloBot - Is the original code for when you have all sensors plugged into the same board.

* ROS Interface for ArloBot WITH 2nd Board - Is the new code I am using now with two boards, and
* 2nd Board Code - Is the code for the Quickstart board.

Please note that while I'm trying to make sure both sets of code are as close as possible to each other, the single board code is not as well testd now. Let me know if you find bugs.

## Customize Code for Your Setup ##
Be sure to adust the following constants in the code based on what sensors you have and where you've plugged them in: (These are not all in one place, you will have to search for them.)

```
const int MCP3208_dinoutPIN = 3;
const int MCP3208_clkPIN = 4;
const int MCP3208_csPIN = 2;
const float referenceVoltage = 5.0;
const int numberOfIRonMC3208 = 6;
const int numberOfPINGsensors = 6;
const int firtPINGsensorPIN = 5;
const int hasGyro = 1;
const int gyroSCLpin = 1;
const int gyroSDApin = 0;
const int hasPIR = 1;
const int PIRpin = 11;
```

There are other things that are not quite so easy to adjust so you may have to tweak the code. If you would like me to make more changes to the code to make it more "adjustable" let me know and I will try.

Ultimately if every Arlo has s different set of PING and Infrared sensors, the code may not be good at handling that.

## ArloBot Libraries Required!##
You will need to have the Parallax ArloBot Libraries in your SimpleIDE workspace in order for this to build. Those libraries are included in this github repository, or you can get them here:
http://forums.parallax.com/showthread.php/152636-Run-ActivityBot-C-Code-in-the-Arlo!?p=1230592&amp;amp;posted=1#post1230592

## Calibration Required!##
Before running this code you must "Calibrate" the ArloDrive routines.
This actually writes a small bit of code into an area of EEPROM that is not erased by future programs that includes tick counts for the motors.

To perform the calibration:

1. Shut off the motors.
2. Put your robot in a safe place, or up on blocks so it can spin the wheels freely,
3. Open: ~/Documents/SimpleIDE/Learn/Simple\ Libraries/Robotics/ActivityBot/libarlocalibrate/libarlocalibrate.side
4. Load this to EEPROM (has to be in EEPROM, not just RAM/memory).
5. Shut off the Activity Board.
6. Turn on the Activity Board.
7. Press and release Activity Board's reset button.
8. Wait a couple seconds. 
9. Turn on motor power and wait for it to try to do something.
10. Press and release Activity Board's reset again after the HB25s are convinced they are getting servo signals. This last step is required only during calibration because it’s crucial to capture all encoder measurements, from the very start of the program.
11. Wait for both wheels to go through a series of movements.
12. Both LEDs will shut off.
13. You are done. Load new code to the Activity Board now.