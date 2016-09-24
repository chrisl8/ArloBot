#include "arlodrive.h"

volatile int abd_trimFL, abd_trimFR, abd_trimBL, abd_trimBR, abd_trimticksF, abd_trimticksB;
volatile int abd_trimticksF;
volatile int abd_trimticksB;
int abd_eeAddr;

void drive_trimSet(int direction, int side, int value)
{
  if(direction >= AB_FORWARD)
  {
    abd_trimticksF = value;
    if(side == AB_LEFT)
    {
      abd_trimFL = 1; 
      abd_trimFR = 0; 
    } 
    else if(side == AB_RIGHT)
    {
      abd_trimFR = 1; 
      abd_trimFL = 0; 
    }
    else 
    {
      abd_trimFL = 0; 
      abd_trimFR = 0;
    }
  }
  else if(direction <= AB_BACKWARD) 
  {
    abd_trimticksB = value;
    if(side == AB_LEFT)
    {
      abd_trimBL = 1; 
      abd_trimBR = 0; 
    } 
    else if(side == AB_RIGHT)
    {
      abd_trimBR = 1; 
      abd_trimBL = 0; 
    }
    else 
    {
      abd_trimBL = 0; 
      abd_trimBR = 0;
    }
  }
  else
  {
    abd_trimBL = 0; abd_trimBR = 0; 
    abd_trimFL = 0; abd_trimFR = 0; 
    abd_trimticksF = 0; abd_trimticksB = 0;
  }
  abd_eeAddr = _ActivityBot_EE_Start_ + _ActivityBot_EE_Trims_;
  ee_putInt(abd_trimFL,     abd_eeAddr +  0);
  ee_putInt(abd_trimFR,     abd_eeAddr +  4);
  ee_putInt(abd_trimBL,     abd_eeAddr +  8);
  ee_putInt(abd_trimBR,     abd_eeAddr + 12);
  ee_putInt(abd_trimticksF, abd_eeAddr + 16);
  ee_putInt(abd_trimticksB, abd_eeAddr + 20);
}

