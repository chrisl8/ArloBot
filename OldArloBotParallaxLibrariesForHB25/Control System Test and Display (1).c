/*
  XBee displays serial updates of calculated and traveled
  ticks with error info.

  XBee DIN -> P11
  IR receiver -> P10

  Use XBee + USB adapter or XStick for PC connection.
  Set PC terminal to 9600 bps.

  Use 2 on TV remote to go 256 ticks forward at top speed
  Use 8 on TV remote to go 256 ticks backward at top speed

  CH+ -> forward, tap for slow, hold to go to full speed.
  CH- -> backward, tap for slow, hold to go to full speed.

  VOL- -> left rotate, tap for slow, hold to go to full speed. 
  VOL+ -> right rotate, tap for slow, hold to go to full speed. 

  Control system variables are in libarlodrive.side.  Open that
  file, then use project manager (bottom left button is show/hide)
  to open arlodrive.c.  

  After changing any of those constants, you have to recompile
  the libarlodrive project for the target memory model (cmm) before
  re-running this test code.

  Control system constants are:
 
  abd_pLo ->   abd_pL = abd_edL * (abd_pLo+(abd_speedL/abd_pLd));  
  abd_pLd  
  abd_iLinc -> if(abd_edL>0)abd_iL+=abd_iLinc; else if(abd_edL<0) abd_iL-=abd_iLinc;
               driveL = abd_iL + abd_pL + ssiL + 1500;

          if(abd_speedL > 0)
            driveL = abd_iL + abd_pL + ssiL + 1500;
          if(abd_speedL < 0)
            driveL = -abd_iL - abd_pL + ssiL + 1500;

  abd_pRo -> abd_pR = abd_edR * (abd_pRo+(abd_speedR/abd_pRd));   
  abd_pRd
  abd_iRinc -> if(abd_edR>0)abd_iR+=abd_iRinc; else if(abd_edR<0) abd_iR-=abd_iRinc;

          if(abd_speedR > 0)
            driveR = -abd_iR - abd_pR + ssiR + 1500;
          if(abd_speedR < 0)
            driveR = abd_iR + abd_pR + ssiR + 1500;
*/                


#include "simpletools.h"
#include "wavplayer.h"                  // Needs 0.9 or later              
#include "sirc.h" 
#include "ping.h"             
#include "arlodrive.h"                  // Needs 0.5.5 or later
#include "fdserial.h"

int key;                                // Global var for remote key
int ticksLeft, ticksLeftCalc, ticksRight, ticksRightCalc; 
int ticksLeftOld, ticksLeftCalcOld, ticksRightOld, ticksRightCalcOld; 
int t, dt, i = 0;
volatile int running = 0;
static int speedLeft, speedRight;

fdserial *xbee;

void gofwd(void *par);
static int fstack[128];
void gobkwd(void *par);
static int bstack[128];

void getTicks();
void displayTicks();

int main()                              // Main - execution begins!
{
  xbee = fdserial_open(11, 10, 0, 9600);
  //dprint(xbee, "hello");
  //dprint(xbee, "hello");

  drive_speed(0,0);                     // Start servos/encoders cog
  drive_setRampStep(10);                // Set ramping at 10 ticks/sec per 20 ms
  sirc_setTimeout(50);                  // Remote timeout = 50 ms

  //drive_feedback(0);

  dt = CLKFREQ/10;
  t  = CNT;

  while(1)                               // Outer loop
  {
    int button = sirc_button(4);      // check for remote key press

    // Motion responses - if key pressed, set wheel speeds
    if(button == 2)
    {
      if(!running) 
        cogstart(&gofwd, NULL, fstack, sizeof fstack);
    }
    if(button == 8)
    {
      if(!running) 
        cogstart(&gobkwd, NULL, bstack, sizeof bstack);
    }
    if(button == CH_UP)drive_rampStep(100, 100); // Left turn      
    if(button == CH_DN)drive_rampStep(-100, -100); // Right turn 
    if(button == VOL_DN)drive_rampStep(-100, 100); // Left turn      
    if(button == VOL_UP)drive_rampStep(100, -100); // Right turn 
    if(button == MUTE)drive_rampStep(0, 0);        // Stop 
    
    if(button == ENTER)
    {
      getTicks();
      displayTicks();
    }

    if(CNT - t > dt)
    {
      t+=dt;
      i++;
      getTicks();
      if
      (
           ticksLeftCalc  != ticksLeftCalcOld 
        || ticksRightCalc != ticksRightCalcOld
        || ticksLeft      != ticksLeftOld
        || ticksRight     != ticksRightOld
      )
      {
        displayTicks();
      }
    }
  }
}

void gofwd(void *par)
{
  running = 1;
  drive_goto(256, 256);
  running = 0;
  cogstop(cogid());
}

void gobkwd(void *par)
{
  running = 1;
  drive_goto(-256, -256);
  running = 0;
  cogstop(cogid());
}


void getTicks(void)
{
  ticksLeftOld = ticksLeft;
  ticksRightOld = ticksRight;
  ticksLeftCalcOld = ticksLeftCalc;
  ticksRightCalcOld = ticksRightCalc;      
  drive_getTicks(&ticksLeft, &ticksRight);
  drive_getTicksCalc(&ticksLeftCalc, &ticksRightCalc);      
  drive_getSpeedCalc(&speedLeft, &speedRight);
}

void displayTicks(void)
{
  writeStr(xbee, "i = ");
  writeDec(xbee, i);
  writeStr(xbee, " eL = ");
  writeDec(xbee, ticksLeftCalc - ticksLeft);
  writeStr(xbee, " eR = ");
  writeDec(xbee, ticksRightCalc - ticksRight);
  writeStr(xbee, "  LSC = ");
  writeDec(xbee, speedRight);
  writeStr(xbee, "  LTC = ");
  writeDec(xbee, ticksLeftCalc);
  writeStr(xbee, " LT = ");
  writeDec(xbee, ticksLeft);
  writeStr(xbee, "  RSC = ");
  writeDec(xbee, speedRight);
  writeStr(xbee, "   RTC = ");
  writeDec(xbee, ticksRightCalc);
  writeStr(xbee, " RT = ");
  writeDec(xbee, ticksRight);
  writeChar(xbee, '\r');
}
