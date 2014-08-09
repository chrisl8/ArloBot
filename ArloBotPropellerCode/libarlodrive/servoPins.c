#include "arlodrive.h"

void interpolation_table_setup(void);

volatile int abd_sPinL, abd_sPinR;   // Global variables
volatile int abd_ePinL, abd_ePinR;
volatile int abd_us;
int abd_intTabSetup;


void drive_servoPins(int servoPinLeft, int servoPinRight)          // drivePins function
{
  //abd_sPinL = servoPinLeft;                                       // Local to global assignments
  //abd_sPinR = servoPinRight;
  //if(!abd_us) abd_us = CLKFREQ/1000000; 

  int eeAddr = _ActivityBot_EE_Start_  + _ActivityBot_EE_Pins_;
  unsigned char pinInfo[8] = {'s', 'p', 'L', 12, ' ', 'R', 13, ' '};  
  pinInfo[3] = (char) servoPinLeft;
  pinInfo[6] = (char) servoPinRight;

  ee_putStr(pinInfo, 8, eeAddr);
  /*
  if(!abd_intTabSetup)
  {
    interpolation_table_setup();
  }
  */
}

void drive_encoderPins(int encPinLeft, int encPinRight)          // drivePins function
{
  //abd_ePinL = encPinLeft;
  //abd_ePinR = encPinRight;
  //if(!abd_us) abd_us = CLKFREQ/1000000; 

  int eeAddr = 8 + _ActivityBot_EE_Start_  + _ActivityBot_EE_Pins_;
  unsigned char pinInfo[8] = {'e', 'p', 'L', 14, ' ', 'R', 15, ' '};  
  pinInfo[3] = (char) encPinLeft;
  pinInfo[6] = (char) encPinRight;

  ee_putStr(pinInfo, 8, eeAddr);

  /*
  if(!abd_intTabSetup)
  {
    interpolation_table_setup();
  }
  */
}

void drive_pins(int servoPinLeft, int servoPinRight, int encPinLeft, int encPinRight)          // drivePins function
{
  drive_servoPins(servoPinLeft, servoPinRight);
  drive_encoderPins(encPinLeft, encPinRight);
}



