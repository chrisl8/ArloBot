Propeller C Code for ArloBot
============================

This is the code to build in SimpleIDE and download to your ArloBot's Activity Board EEPROM in order to communicate with ROS over the USB serial interface.

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

## ArloBot Libraries Required!##
You will need to have the Parallax ArloBot Libraries in your SimpleIDE workspace in order for this to build. Those libraries are included in this github repository, or you can get them here:
http://forums.parallax.com/showthread.php/152636-Run-ActivityBot-C-Code-in-the-Arlo!?p=1230592&amp;amp;posted=1#post1230592
