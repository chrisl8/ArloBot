Propeller C Code for ArloBot
============================

This is the code to build in SimpleIDE and download to your ArloBot's Activity Board EEPROM in order to communicate with ROS over the USB serial interface.
Details on the Propeller code and setup are here: [http://ekpyroticfrood.net/?p=165](http://ekpyroticfrood.net/?p=165)

## Personal Settings ##
The files in the folder 'dotfiles' should be in
~/.arlobot/
where SimpleIDE will look for them when building "ROS Interface for ArloBot.c"

I suggest NOT editing the file in 'dotfiles', but always edit the copy in
~/.arlobot/ so that future git pulls will not damage your
personal edits to this file.

## One Version Now ##
Note, I now have a second Propeller based Quickstart board on my Arlo. All PING and Infrared sensors are on this board. This means I have two versions of the code here.
* ROS Interface for ArloBot - This will work in all cases.

* 2nd Board Code - Is the code for the Quickstart board, if you have one, if not, ignore it.

```
## Customize Code for Your Setup ##

You have to open the code in SimpleIDE to install it on your Propeller Activity Board.
When you do, read the top of the file and comment out "#define has..." lines for things you do not have,
and adjust the numbers for things you do have.
The entire thing should be documented in the code at the top, and if not, let me know.

Once you set all of the #define lines properly, the code should work on your setup.

I decided to use #define because these settings occur BEFORE the compiler compiles the code. So anything you set as "not included" is never even compiled, saving memory on the microcontroller.

## ArloBot Libraries Required!##
You will need to have the Parallax ArloBot Libraries in your SimpleIDE workspace in order for this to build. Those libraries are included in this github repository, or you can get them here:
http://forums.parallax.com/showthread.php/152636-Run-ActivityBot-C-Code-in-the-Arlo!?p=1230592&amp;amp;posted=1#post1230592

## Connecting the encoders
The encoders are supposed to be connected to the DHB10 motor board.
However, we have experienced regular crashs of the motor board when encoders are connected to it.
Therefore, we have made a system that allows encoders to be either connected to the motor board,
or connected to the Propeller Activity Board WX, like:
* Encoder left A to pin 3
* Encoder left B to pin 2
* Encoder right A to pin 1
* Encoder right B to pin 0

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
