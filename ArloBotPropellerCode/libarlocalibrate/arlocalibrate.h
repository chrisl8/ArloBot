/**
 * @file arlocalibrate.h
 *
 * @author Andy Lindsay
 *
 * @version 0.5
 *
 * @copyright Copyright (C) Parallax, Inc. 2013.  See end of file for
 * terms of use (MIT License).
 *
 * @brief This library has a function you can call to calibrate your 
 * Arlo.  
 * <br>
 * <br>
 * Calibration instructions that accompany the example code are included in the tutorial.
 *
 * Please submit bug reports, suggestions, and improvements to
 * this code to editor@parallax.com.
 */

#ifndef ABCALIBRATE_H
#define ABCALIBRATE_H

#if defined(__cplusplus)
extern "C" {
#endif

#include "servo.h"
#include "simpletools.h"                      // Include simple tools
#include "fdserial.h"


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

/**
 * @brief Run the ActivityBot calibratino function.  Let it run until the
 * P26 and P27 lights turn off.  It'll take about 1 minute, 20 seconds.
 */ 
void cal_arlo();

#if defined(__cplusplus)
}
#endif
/* __cplusplus */  
#endif
/* ABCALIBRATE_H */  

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
