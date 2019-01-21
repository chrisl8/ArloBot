#include "arlodrive.h"

void interpolation_table_setup(void);
void set_drive_speed(int left, int right);

volatile unsigned int _servoPulseReps;
volatile int abd_speedL;      // Requested servo speed left
volatile int abd_speedR;      // Requested servo speed right
int abd_rampStep;
int abd_intTabSetup;


//static int step = 6;
void drive_rampStep(int left, int right)
{
  int leftTemp, rightTemp;
  int sprOld = _servoPulseReps;

  if(left > abd_speedL + abd_rampStep) leftTemp = abd_speedL + abd_rampStep;
  else if(left < abd_speedL - abd_rampStep) leftTemp = abd_speedL - abd_rampStep;
  else leftTemp = left;

  if(right > abd_speedR + abd_rampStep) rightTemp = abd_speedR + abd_rampStep;
  else if(right < abd_speedR - abd_rampStep) rightTemp = abd_speedR - abd_rampStep;
  else rightTemp = right;

  drive_speed(leftTemp, rightTemp);
  while(sprOld >= _servoPulseReps);
  sprOld = _servoPulseReps;
}


//static int step = 6;
void drive_ramp(int left, int right)
{
  if(!abd_intTabSetup)
  {
    interpolation_table_setup();
    set_drive_speed(0, 0);
    pause(40);
  }
  
  while((left != abd_speedL) || (right != abd_speedR))
  {
    drive_rampStep(left, right);
  }
}

