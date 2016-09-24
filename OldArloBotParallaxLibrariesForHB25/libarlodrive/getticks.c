#include "arlodrive.h"

volatile int abd_ticksL;
volatile int abd_ticksR;
volatile int abd_dlc;         // distance left calculated
volatile int abd_drc;         // distance right calculated
volatile int abd_speedL;      // Requested servo speed left
volatile int abd_speedR;      // Requested servo speed right

void drive_getTicks(int *left, int *right)
{
  *left = abd_ticksL;
  *right = abd_ticksR;
}

void drive_getTicksCalc(int *left, int *right)
{
  *left = abd_dlc;
  *right = abd_drc;
}

void drive_getSpeedCalc(int *left, int *right)
{
  *left  = abd_speedL;
  *right = abd_speedR;
}

