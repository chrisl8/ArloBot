//#define interactive_development_mode

#include "arlodrive.h"
#include "fdserial.h"

volatile int abd_record;               // Record values to an array

volatile int abd_elCntL;
volatile int abd_elCntR;
volatile int abd_cntrLidx;
volatile int abd_cntrRidx;

int abd_spdrL[120];
int abd_spdmL[120];
int abd_spdrR[120];
int abd_spdmR[120];

int abd_intTabSetup;


#ifdef interactive_development_mode
volatile int abd_rv[2600];
fdserial *xbee;
volatile int xbee_setup;
int abd_intTabSetup;
volatile int abd_trimFL, abd_trimFR, abd_trimBL, abd_trimBR, abd_trimticksF, abd_trimticksB;
volatile int abd_eaL;
volatile int abd_eaR;
volatile int abd_ridx;
//volatile int abd_rv[2600];
void interpolation_table_setup();
#endif // interactive_development_mode



#ifdef interactive_development_mode
void drive_displayInterpolation(void)
{
  if(!xbee_setup)
  {
    xbee = fdserial_open(11, 10, 0, 9600);
    xbee_setup = 1;
  }

  if(!abd_intTabSetup) interpolation_table_setup();
 
  dprint(xbee, "Left Servo\n");
  dprint(xbee, "abd_elCntL = %d, abd_cntrLidx = %d\n\n", abd_elCntL, abd_cntrLidx);
  for(int r = 0; r < abd_elCntL; r++)
  {
    dprint(xbee, "r = %d, spdrL = %d, spdmL = %d, \n", r, abd_spdrL[r], abd_spdmL[r]);
  }
  
  dprint(xbee, "Right Servo\n");
  dprint(xbee, "\n\nelCntR = %d, cntrRidx = %d\n\n", abd_elCntR, abd_cntrRidx);
  for(int r = 0; r < abd_elCntR; r++)
  {
    dprint(xbee, "r = %d, spdrR = %d, spdmR = %d, \n", r, abd_spdrR[r], abd_spdmR[r]);
  }
  //getchar();  
  //#endif
}
#endif // interactive_development_mode


#ifdef interactive_development_mode
void drive_trimDisplay(void)
{
  if(!xbee_setup)
  {
    xbee = fdserial_open(11, 10, 0, 9600);
    xbee_setup = 1;
  }

  if(!abd_intTabSetup) interpolation_table_setup();

  dprint(xbee, "trimFL %d, trimFR %d, trimBL %d, trimBR %d, trimticksF %d, trimticksB %d\n",
         abd_trimFL, abd_trimFR, abd_trimBL, abd_trimBR, abd_trimticksF, abd_trimticksB);
}
#endif // interactive_development_mode


#ifdef interactive_development_mode
void drive_displayControlSystem(int start, int end)
{
  if(!xbee_setup)
  {
    xbee = fdserial_open(11, 10, 0, 9600);
    xbee_setup = 1;
  }

  dprint(xbee, "\neaL = %d, eaR = %d \n\n", abd_eaL, abd_eaR);
  dprint(xbee, "ridx = %d \n\n", abd_ridx);
  for(int i = start*11; i < (end-1) * 11; i+=11)
  {
    //dprint(xbee, "index = %d, speed = %d, dlc = %d, ticksL = %d, edL = %d, drc = %d, ticksR = %d, edr = %d\n", 
    //      i, rv[i],  rv[i+1], rv[i+2], rv[i+3], rv[i+4], rv[i+5], rv[i+6]); 
    dprint(xbee, "i = %d, v = %d, dlc = %d, ticksL = %d, edL = %d, pL = %d, iL = %d, drc = %d, ticksR = %d, edr = %d, pR = %d, iR = %d\n", 
          i, abd_rv[i],  abd_rv[i+1], abd_rv[i+2], abd_rv[i+3], abd_rv[i+4], abd_rv[i+5], abd_rv[i+6], abd_rv[i+7], abd_rv[i+8], abd_rv[i+9], abd_rv[i+10]); 
  }
  dprint(xbee, "\neaL = %d, eaR = %d \n\n", abd_eaL, abd_eaR);
}
#endif // interactive_development_mode


#ifndef interactive_development_mode
fdserial *xbee;
#endif


#ifdef interactive_development_mode
void drive_record(int startStop)
{
  abd_record = startStop;
}
#endif

