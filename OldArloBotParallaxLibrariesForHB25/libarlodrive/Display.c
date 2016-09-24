//#define interactive_development_mode

#include "arlodrive.h"

void interpolation_table_setup();

int abd_intTabSetup;
volatile int abd_trimFL, abd_trimFR, abd_trimBL, abd_trimBR, abd_trimticksF, abd_trimticksB;

volatile int abd_elCntL;
volatile int abd_elCntR;
volatile int abd_cntrLidx;
volatile int abd_cntrRidx;

int abd_spdrL[120];
int abd_spdmL[120];
int abd_spdrR[120];
int abd_spdmR[120];

#ifdef interactive_development_mode
volatile int abd_rv[2600];
fdserial *xbee;
volatile int xbee_setup;
int abd_intTabSetup;
volatile int abd_trimFL, abd_trimFR, abd_trimBL, abd_trimBR, abd_trimticksF, abd_trimticksB;
volatile int abd_eaL;
volatile int abd_eaR;
volatile int abd_ridx;
#endif // interactive_development_mode

#ifndef interactive_development_mode
void drive_displayInterpolation(void)
{
  if(!abd_intTabSetup) interpolation_table_setup();
 
  print("=== LEFT SERVO ===\n\n");
  print("Table Entries = %d\nZero Speed Index = %d\n\n", abd_elCntL, abd_cntrLidx);
  print("Index\tServo Drive\tEncoder Ticks/Second\n");
  print("-----\t-----------\t--------------------\n");
  for(int r = 0; r < abd_elCntL; r++)
  {
    print("%d\t%d\t\t%d\n", r, abd_spdrL[r], abd_spdmL[r]);
  }
  
  print("\n\n=== RIGHT SERVO ===\n\n");
  print("Table Entries = %d\nZero Speed Index = %d\n\n", abd_elCntR, abd_cntrRidx);
  print("Index\tServo Drive\tEncoder Ticks/Second\n");
  print("-----\t-----------\t--------------------\n");
  for(int r = 0; r < abd_elCntR; r++)
  {
    print("%d\t%d\t\t%d\n", r, abd_spdrR[r], abd_spdmR[r]);
  }
  
  //getchar();  
  //#endif
}
#endif // interactive_development_mode


#ifndef interactive_development_mode
void drive_trimDisplay(void)
{
  if(!abd_intTabSetup) interpolation_table_setup();

  print("trimFL %d, trimFR %d, trimBL %d, trimBR %d, trimticksF %d, trimticksB %d\n",
         abd_trimFL, abd_trimFR, abd_trimBL, abd_trimBR, abd_trimticksF, abd_trimticksB);
}
#endif // interactive_development_mode


