/* ATTENTION! ATTENTION! ATTENTION! ATTENTION! ATTENTION! ATTENTION!
You MUST edit the settings in
~/.arlobot/per_robot_settings_for_propeller_2nd_board.h
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
By using the #define lines, code for items you do not have is never seen by the compiler and is never even loaded on the Propeller bard, saving memory. */

#include "per_robot_settings_for_propeller_2nd_board.h"
/* If SimpleIDE build fails because the above file is missing,
open up the "Project Manager", then the "Compiler" tab,
and fix the path to your ~/.arlobot/ folder
under Other Compiler Options
and/or copy the above file from the dotfiles folder
to your ~/.arlobot folder.
You could also just move the files in the dotfiles folder into
the folder with this file, but future "git pull" updates
will erase your changes.*/

/* 2nd Propeller Board (QuickStart Board) Code for ArloBot

   This code should run on the 2nd board and send data to the primary
   Activity Board via two pins using fdserial.
*/

#include "simpletools.h"
#include "mcp3208.h" // MCP3208 8 Chanel ADC
#include "ping.h" // Include ping header
#include "fdserial.h" // http://learn.parallax.com/propeller-c-simple-protocols/full-duplex-serial
// and http://propsideworkspace.googlecode.com/hg-history/daf5de8bf52840e02d5615edaa6d814e59d1b0b0/Learn/Simple%20Libraries/Text%20Devices/libfdserial/html/fdserial_8h.html#ab14338477b0b96e671aed748e20ecf5e

//fdserial *term;
fdserial *propterm;

int mcp3208_IR_cm(int); // Function to get distance in CM from IR sensor using MCP3208

void pollPingSensors(void *par); // Use a cog to fill range variables with ping distances
static int pstack[128]; // If things get weird make this number bigger!
int isActive = 0;
int main()
{

  // Close the simpleterm half duplex connection
  simpleterm_close();

  // Start the sensor cog(s)
	cogstart(&pollPingSensors, NULL, pstack, sizeof pstack);

  // Blinken Lights!
  const int pauseTime = 50;
  const int startLED = 17; // 16 is used for the pollPingSensors cog to indicate activity.
  const int endLED = 23;
  high(startLED);
  pause(pauseTime);
  low(startLED);
  while(1) {
  pause(5);
  if(isActive == 1) {
      for (int led = startLED + 1; led <= endLED; led++) {
        high(led);
        pause(pauseTime);
        low(led);
      }
      for (int led = endLED - 1; led >= startLED; led--) {
        high(led);
        pause(pauseTime);
        low(led);
      }
    isActive = 0;
    }
  }
}

void pollPingSensors(void *par) {
  // The last IR sensor will be retagged with this position number,
  // in case there are more PINGs than IRs.
  const int lastIRposition = 7;
  propterm = fdserial_open(QUICKSTART_RX_PIN, QUICKSTART_TX_PIN, 0, 115200);
  //term = fdserial_open(31, 30, 0, 115200); // for Debugging
  // Initialize variables outside of loop
  // This may or may not improve performance.
  int ping = 0, ir = 0;
  char receivedChar;
 while(1) // Repeat indefinitely
  {
    /* We wait for input from the other side,
       Which lets us not activate the sensors
       if the other end is not working,
       and also lets the other end rate limit the input. */
    receivedChar = fdserial_rxChar(propterm);
    //char receivedChar = 'i'; // for Debugging - Cause it to always run instead of waiting for a signal
    // Only send data when we get the expected "init" character, avoiding running on random garbage from an open connection
    if (receivedChar == 'i') {
      high(16); // LEDs for debugging
      isActive = 1;
      for(int i=0; i < NUMBER_OF_PING_SENSORS; i++ ) {
        ping = ping_cm(FIRST_PING_SENSOR_PIN + i);
        dprint(propterm, "p,%d,%d.", i, ping);
        receivedChar = fdserial_rxChar(propterm); // Should get a character after each output for rate limiting
        //dprint(term, "p,%d,%d\n", i, ping); // For Debugging
        if(i < NUMBER_OF_IR_SENSORS) { // If there is also an IR sensor at this number check it too
          ir = mcp3208_IR_cm(i);
          dprint(propterm, "i,%d,%d.", i, ir);
          receivedChar = fdserial_rxChar(propterm); // Should get a character after each output for rate limiting
        }
      }
     //dprint(term, "\n"); // For Debugging - add a line break here and pull the above two
    }
    low(16); // LEDs for debugging
  }
/* TODO: Be sure to test that the data coming in is REAL TIME! */
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
