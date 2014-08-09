/* 

  Arlo Calibrate.c

  Calibrate the Arlo's motors and encoders

*/

#include "simpletools.h"
#include "arlocalibrate.h"    

int main()
{
  high(26);
  high(27);
  cal_arlo();
  low(26);
  low(27);
}
