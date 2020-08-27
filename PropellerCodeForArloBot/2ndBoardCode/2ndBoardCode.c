/* ATTENTION! ATTENTION! ATTENTION! ATTENTION! ATTENTION! ATTENTION!
NOTE: This code is for the DHB-10 Motor Controller that comes with the new Parallax Arlo kit.
You MUST edit the settings in
~/.arlobot/per_robot_settings_for_propeller_c_code.h
based on the physical layout of your robot!
For each QUESTION:
UNCOMMENT '#define' lines for any included items,
COMMENT '#define' lines for anything that is not included.
For each SETTING:
Set the variable as required, noting that usually these are ignored if the preceding QUESTION is commented out.

Example, My robot has a "Thing1", but not a "Thing2"
*/
#define hasThingOne
//#define hasTHingTwo

/* Just like that, comment out the "has" line for things you do not have,
and if you do have the thing, adjust the numbers on the other definition as needed.
By using the #define lines, code for items you do not have is never seen by the compiler and is never even loaded on the
Propeller board, saving memory and avoiding any errors or crashes caused by unused code. */

#include "per_robot_settings_for_propeller_c_code.h"
/* If SimpleIDE build fails because the above file is missing,
open up the "Project Manager", then the "Compiler" tab,
and fix the path to your ~/.arlobot/ folder
under Other Compiler Options
and/or copy the above file from the dotfiles folder
to your ~/.arlobot folder.
You could also just move the files in the dotfiles folder into
the folder with this file, but future "git pull" updates
may erase your changes.*/

/* 2nd Propeller Board (QuickStart Board) Code for ArloBot

   This code should run on the 2nd board and send data to the primary
   Activity Board via two pins using fdserial.
*/

#include "simpletools.h"
#include "mcp3208.h" // MCP3208 8 Chanel ADC
#include "ping.h" // Include ping header
#include "fdserial.h" // http://learn.parallax.com/propeller-c-simple-protocols/full-duplex-serial
// and http://propsideworkspace.googlecode.com/hg-history/daf5de8bf52840e02d5615edaa6d814e59d1b0b0/Learn/Simple%20Libraries/Text%20Devices/libfdserial/html/fdserial_8h.html#ab14338477b0b96e671aed748e20ecf5e

#ifdef hasButtons
static int buttonStatus[4] = {0};
void pollButtons(void *par);
static int pollButtonsStack[120]; // If things get weird make this number bigger.
#endif

fdserial *propterm;

int mcp3208_IR_cm(int); // Function to get distance in CM from IR sensor using MCP3208

void pollPingSensors(void *par); // Use a cog to fill range variables with ping distances
static int pstack[256]; // If things get weird make this number bigger!
int main()
{

  // Close the simpleterm half duplex connection
  simpleterm_close();

  // Start the sensor cog(s)
	cogstart(&pollPingSensors, NULL, pstack, sizeof pstack);

#ifdef hasButtons
    // Start the button poling cog
    cogstart(&pollButtons, NULL, pollButtonsStack, sizeof pollButtonsStack);
#endif
}

int checkCharacter() {
  char receivedChar = fdserial_rxChar(propterm);
  if (receivedChar == 'i') {
    return 1;
  }
  if (receivedChar == 'l') {
        for (int i=0; i < NUMBER_OF_LEDS; i++) {
            receivedChar = fdserial_rxChar(propterm);
            if (receivedChar == '1') {
                high(20 + i);
            } else {
                low(20 + i);
            }
        }
    }
    return 0;      
}  

void pollPingSensors(void *par) {
  // The last IR sensor will be retagged with this position number,
  // in case there are more PINGs than IRs.
  propterm = fdserial_open(ACTIVITYBOARD_RX_PIN, ACTIVITYBOARD_TX_PIN, 0, 115200);
  //propterm = fdserial_open(31, 30, 0, 115200); // for Debugging send data to USB Serial
  char receivedChar;
 while(1) {// Repeat indefinitely
    /* We wait for input from the other side,
       Which lets us not activate the sensors
       if the other end is not working,
       and also lets the other end rate limit the input. */
    
    //char receivedChar = 'i'; // for Debugging - Cause it to always run instead of waiting for a signal
    // Only send data when we get the expected "init" character, avoiding running on random garbage from an open connection
    if (checkCharacter() == 1) {
      for(int i=0; i < NUMBER_OF_PING_SENSORS; i++ ) {
        int ping = ping_cm(FIRST_PING_SENSOR_PIN + i);
        dprint(propterm, "p,%d,%d.", i, ping);
        checkCharacter(); // Should get a character "i" after each output for rate limiting
        if(i < NUMBER_OF_IR_SENSORS) { // If there is also an IR sensor at this number check it too
          int ir = mcp3208_IR_cm(i);
          dprint(propterm, "i,%d,%d.", i, ir);
          checkCharacter(); // Should get a character "i" after each output for rate limiting
        }
      }
#ifdef hasButtons
        for (int i = 0; i < 4; i++) {
            if (buttonStatus[i] == 1) {
                dprint(propterm, "b,%d.", i);
                buttonStatus[i] = 0;
                checkCharacter(); // Should get a character "i" after each output for rate limiting
            }
        }
#endif
#ifdef hasLEDs
        dprint(propterm, "l."); // Now tell Activity board to send us the LEDs
        #endif
    }
  }
}

int mcp3208_IR_cm(int channel) {
const int MCP3208_dinoutPIN = 27;
const int MCP3208_clkPIN = 25;
const int MCP3208_csPIN = 26;
const float referenceVoltage = 5.0; // MCP3208 reference voltage setting. I use 5.0v for the 5.0v IR sensors from Parallax
  int mcp3208reading  = readADC(channel, MCP3208_dinoutPIN, MCP3208_clkPIN, MCP3208_csPIN);
  float mcp3208volts = (float)mcp3208reading * referenceVoltage / 4096.0;
  int mcp3208cm = 27.86 * pow(mcp3208volts, -1.15); // https://www.tindie.com/products/upgradeindustries/sharp-10-80cm-infrared-distance-sensor-gp2y0a21yk0f/
  return(mcp3208cm);
}

#ifdef hasButtons
void pollButtons(void *par) {
    while (1) {
        pause(100);
        for (int i = 0; i < 4; i++) {
          if (input(16 + i) == 1) {
            buttonStatus[i] = 1;
          }            
        }
    }
}
#endif
