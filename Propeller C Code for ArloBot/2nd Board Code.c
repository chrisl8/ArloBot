/* 2nd Propeller Board (QuickStart Board) Code for ArloBot

   This code should run on the 2nd board and send data to the primary
   Activity Board via two pins using fdserial.
*/

#include "simpletools.h"
#include "ping.h"                             // Include ping header
#include "fdserial.h" // http://learn.parallax.com/propeller-c-simple-protocols/full-duplex-serial
// and http://propsideworkspace.googlecode.com/hg-history/daf5de8bf52840e02d5615edaa6d814e59d1b0b0/Learn/Simple%20Libraries/Text%20Devices/libfdserial/html/fdserial_8h.html#ab14338477b0b96e671aed748e20ecf5e

fdserial *term;
fdserial *propterm;

const int numberOfPINGsensors = 10;
const int firtPINGsensorPIN = 2; // Which pin the first PING sensor is on
const int propRXpin = 0;
const int propTXpin = 1;
void pollPingSensors(void *par); // Use a cog to fill range variables with ping distances
static int pstack[128]; // If things get weird make this number bigger!
int isActive = 0;
int main()                    
{

  // Close the simpleterm half duplex connection
  simpleterm_close(); // The simpleterm_close function call shuts down the default half-duplex communication, which will cause print calls to stop working.  But that’s fine because we can use full-duplex serial.
  // Start a full duplex terminal session
  //term = fdserial_open(31, 30, 0, 115200);
    // Start the sensor cog(s)
	cogstart(&pollPingSensors, NULL, pstack, sizeof pstack);
    
    // Blinken Lights!
  const int pauseTime = 50;
  const int startLED = 17; // 16 is used for the pollPingSensors cog to indicate activity!
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
  const int pingDelay = 10;
  propterm = fdserial_open(propRXpin, propTXpin, 0, 115200);
 while(1)                                    // Repeat indefinitely
  {
    /* We wait for ANY input from the other side,
       Which lets us not activate the sensors if the other end is not working,
       and also lets the other end rate limit the input */
    char receivedChar = fdserial_rxChar(propterm);
    if (receivedChar == 'i') { // Only send data when we get the expected "init" character, avoiding running on random garbage from an open connection
      high(16); // LEDs for debugging
      isActive = 1;
      for(int i=0; i < numberOfPINGsensors; i++ ) {
        dprint(propterm, "p,%d,%d\n", i, ping_cm(firtPINGsensorPIN + i));
        pause(pingDelay); // Otherwise it is just stupid fast!
          //pingArray[i] = ping_cm(firtPINGsensorPIN + i);
          //irArray[i] = mcp3208_IR_cm(i);
      }
    }
    low(16); // LEDs for debugging
    fdserial_rxFlush(propterm); // In case we got a flood for some reason, flush it.
  }
}
/* Be sure to test that the data coming in is REAL TIME! */
