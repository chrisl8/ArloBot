//#define interactive_development_mode

/**
 * @file arlodrive.h
 *
 * @author Andy Lindsay
 *
 * @copyright Copyright (C) Parallax, Inc. 2013.  See end of file for
 * terms of use (MIT License).
 *
 * @version 0.5.5
 *
 * @b v0.5.5
 * @li drive_getTicksCalc
 * @li drive_getTicks
 * @li drive_open
 * @li drive_encoderPins
 * @li drive_servoPins
 * @li Values outside interpolation table ranges do not result in
 * rotation halt. 
 * @li Turning off feedback now allows full servo speed operation
 *
 * @b v0.5.1
 * @li Trim enabled by default.
 * @li Clear trim settings during calibration. (v0.5.1)
 * @li Make trim for a direction mutually exclusive to one side. (v0.5.1)
 *
 * @b To-Do
 * @li Make ramp go to percentage of full speed
 * @li Corrrect error in start string verification
 * @li Make sure that control system's compensation cannot cause the servo signal to cross
 * its direction boundary  
 * @li drive_distance
 * @li drive_getSpeedCalc
 * @li drive_getSpeedActual
 * @li Code comments
 * @li Reduce calibration array elements to 16-bit
 * @li Condense left/right duplicate code
 * @li Remove other variables
 *
 * @brief This library takes care of encoder monitoring and servo signalling, 
 * and provides a simple set of functions for making the Arlo go certain
 * distances and speeds.
 * <br>
 * <br>
 * For more information, go here:
 * <br>
 * <br>
 * Please submit bug reports, suggestions, and improvements to
 * this code to editor@parallax.com.
 */

#ifndef ABDRIVE_H
#define ABDRIVE_H

#if defined(__cplusplus)
extern "C" {
#endif

#include "simpletools.h"
#include "simpletext.h"
#include "fdserial.h"

/*
 #ifndef interactive_development_mode
 #define interactive_development_mode
 void display_control_sys(int start, int end);
 #endif
*/

#ifndef _ActivityBot_EE_Start_
/**
 *
 * @brief ActivityBot EEPROM calibration data start address.
 */
#define _ActivityBot_EE_Start_ 63418
#endif

#ifndef _ActivityBot_EE_Pins_
#define _ActivityBot_EE_Pins_ 12
#endif

#ifndef _ActivityBot_EE_Trims_
#define _ActivityBot_EE_Trims_ 28
#endif

#ifndef _ActivityBot_EE_Left_
#define _ActivityBot_EE_Left_ 52
#endif

#ifndef _ActivityBot_EE_Right_
#define _ActivityBot_EE_Right_ 1052
#endif

#ifndef _ActivityBot_EE_End_
/**
 *
 * @brief ActivityBot EEPROM calibration data end address.
 */
#define _ActivityBot_EE_End_ 63418 + 2052
#endif

#ifndef AB_RIGHT  
#define AB_RIGHT  1
#endif

#ifndef AB_LEFT
#define AB_LEFT -1
#endif


#ifndef AB_FORWARD
#define AB_FORWARD 1
#endif

#ifndef AB_BACKWARD
#define AB_BACKWARD -1
#endif

#ifndef OFF
/**
 * @brief OFF can be used in place of zero to enabled parameters in
 * functions like drive_feedback and drive_trim.
 */
#define OFF 0
#endif

#ifndef ON
/**
 * @brief ON can be used in place of a nonzero value to enabled
 * parameters in functions like drive_feedback and drive_trim.
 */
#define ON  1
#endif


/**
 * @brief Enables or disables encoder feedback for speed control.  
 *
 * @param enabled Set to 1 to enable feedback (default) or 0 to disable.
 */
void drive_feedback(int enabled);                      


/**
 * @brief Enables or disables drive trim which can be used to compensate for
 * mechanical wheel alignment errors  
 *
 * @param enabled Set to 1 to enable trim (default) or 0 to disable.
 */
void drive_trim(int enabled);


/**
 * @brief Stores trim values to EEPROM.
 *
 * @details Trim values can compensate for mechanical wheel alignment errors. 
 * When you set the Arlo's trim, you are telling it to make a certain
 * wheel turn an extra tick per certain number of ticks.  For example, you
 * can use this to make the right wheel turn 1 extra tick per every 64 ticks.
 * It will actually just expect to see an encoder transition 1/64th of a tick
 * sooner with every tick. 
 *
 * @param direction Selects to set the trim for a given direction.  Use
 * AB_FORWARD OR AB_BACKWARD.
 *
 * @param side Selects the side to make one more or less ticks per number of
 * ticks.  Use AB_LEFT or AB_RIGHT to select the left or right wheel.
 *
 * @param value Number of ticks that should elapse before the extra tick will
 * have accumulated.  Use a negative number if you want a given wheel to go
 * one less tick per number of ticks instead of one more tick.  
 */
void drive_trimSet(int direction, int side, int value);


/**
 * @brief Display the trim settings
 *
 * @details Displays most recent direction, side, and value settings from the
 * most recent call to drive_trimSet.
 */
void drive_trimDisplay(void);


/**
 * @brief Displays the interopolation table stored in EEPROM by the calibration
 * step.  For more info, see:
 */
void drive_displayInterpolation(void);


/**
 * @brief Set encoder pins to values other than the default P14 for left 
 * encoder and P15 for right encoder.  Stores values in EEPROM, so you only
 * need to call this function at the start of one program.  Programs that are 
 * after that will get the values from EEPROM.
 *
 * @param encPinLeft I/O pin number for the left encoder signal connection.
 *
 * @param encPinRight I/O pin number for the right encoder signal connection.
 */
void drive_encoderPins(int encPinLeft, int encPinRight);


/**
 * @brief Set servo pins to values other than the default P12 for left 
 * servo and P13 for right servo.  Stores values in EEPROM, so you only
 * need to call this function at the start of one program.  Programs that are 
 * after that will get the values from EEPROM.
 *
 * @param servoPinLeft I/O pin number for the left servo signal connection.
 *
 * @param servoPinRight I/O pin number for the right servo signal connection.
 */
void drive_servoPins(int servoPinLeft, int servoPinRight);


/**
 * @brief Set wheel speeds in encoder ticks per second.  An encoder tick is
 * 1/64th of a revolution, and makes causes the wheel to roll 3.25 mm.
 *
 * @param left Left wheel speed in ticks per second.
 *
 * @param right Left wheel speed in ticks per second.
 */
void drive_speed(int left, int right); 


/**
 * @brief Ramp up to the specified wheel speeds.  It works almost the same as
 * drive_speed, except that it steps up the speed every 50th of a second.  The
 * default ramping rate is 4 ticks per 50th of a second.  This function will 
 * make your program wait while it ramps.
 *
 * @param left Left wheel speed in ticks per second.
 *
 * @param right Left wheel speed in ticks per second.
 */
void drive_ramp(int left, int right);


/**
 * @brief This funciton allows your code to ask for a speed repeatedly in a loop, 
 * but each time your code asks for that speed, it takes a step toward the speed.
 * This helps cushon sudden maneuvers in sensor navigation, where the conditions
 * might change more rapidly than you would want your Arlo's speed to 
 * change.
 * 
 * @param left Left wheel speed in ticks per second.
 *
 * @param right Left wheel speed in ticks per second.
 */
void drive_rampStep(int left, int right);

/**
 * @brief Overrides the default 4 ticks/second per 50th of a second for ramping.
 *
 * @param stepsize The size of each step in ticks/second to change every 50th of
 * a second
 */
void drive_setRampStep(int stepsize);


/**
 * @brief Make the Arlo wheels roll certain encoder tick distances.  An 
 * encoder tick is 1/64th of a wheel turn and makes the wheel roll 3.25 mm.  
 * NOT YET IMPLEMENTED, use drive_goto instead.
 *
 * @param left Left wheel encoder tick distance.
 *
 * @param right Right wheel encoder tick distance.
 */
void drive_distance(int left, int right);


/**
 * @brief Stop the servo/encoder system.  This is useful for reclaiming a processor 
 * for other tasks.
 */
void drive_close(void);



/**
 * @brief Start or restart the servo/encoder system.  
 */
int  drive_open();


/**
 * @brief Modifies the default maxiumum top speed for use with encoders.  The default
 * is 128 ticks/second = 2 revolutions per second (RPS).  This is the full speed that
 * drive_distance and drive_goto use.  This value can currently be reduced, but not 
 * increased.  Speeds faster than 128 ticks per second are "open loop" meaning the control
 * system does not use the encoders to correct distance/speed. 
 *
 * @param speed Maximum cruising speed for drive_distance and drive_goto.
 */
void drive_setMaxSpeed(int speed);


/**
 * @brief Make each wheel go a particular distance.  Recommended for straight forward,
 * backward, turns and pivots.  Not recommended for curves.  This function ramps up to
 * full speed if the distance is long enough.  It holds that speed until it needs to 
 * ramp down.  After ramping down it applies compensation.  This function is primarily a 
 * convenience for dead reckoning, and does not return until the maneuver has completed. 
 *
 * @param distLeft Left wheel distance.
 *
 * @param distRight Right wheel distance.
 */
void drive_goto(int distLeft, int distRight);


/**
 * @brief Get the measured number of ticks the have traveled. 
 *
 * @details The system samples the encoders at 400 times per second.
 *
 * @param *left Pointer to variable to receive the measured left distance.  
 *
 * @param *right Pointer to variable to receive the measured right distance.  
 */
void drive_getTicks(int *left, int *right);


/**
 * @brief Get the calculated number of ticks the encoders should have traveled. 
 *
 * @details The system samples the encoders at 400 times per second.
 *
 * @param *left Pointer to variable to receive the calculated left distance.  
 *
 * @param *right Pointer to variable to receive the calculated right distance.  
 */
void drive_getTicksCalc(int *left, int *right);


/**
 * @brief Get the calculate number of ticks the encoders should have traveled.  Distance
 * is in ticks.  The wheel has 64 encoder ticks, each accounting for 3.25 mm of distance.
 *
 * @details The system updates the predicted distance 400 times per second, based on the
 * speed the wheels should be going.
 *
 * @param *left Pointer to variable to receive the calcualted left distance.  
 *
 * @param *right Pointer to variable to receive the calcualted right distance.  
 */
void drive_getTicksCalc(int *left, int *right);


/**
 * @brief Get the calculated speed in ticks per second.
 *
 * @details Some functions work in by changing the speed over time.  You can
 * use this function to find out what speed the system is asking for at a 
 * given time.
 *
 * @param *left Pointer to variable to receive the calcualted left speed.  
 *
 * @param *right Pointer to variable to receive the calcualted right speed.  
 */
void drive_getSpeedCalc(int *left, int *right);


/*
  void drive_getSpeedCalc(int *left, int *right);
  void drive_getSpeedActual(int *left, int *right);
*/

#ifdef interactive_development_mode
void drive_displayControlSystem(int start, int end);
void drive_record(int startStop);
void drive_interpolation_table_setup();
#endif

#if defined(__cplusplus)
}
#endif
/* __cplusplus */  
#endif
/* ABDRIVE_H */  

/**
 * TERMS OF USE: MIT License
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 * DEALINGS IN THE SOFTWARE.
 */
