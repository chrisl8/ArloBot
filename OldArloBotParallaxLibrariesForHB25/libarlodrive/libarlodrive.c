//#define interactive_development_mode

/*
 * @brief Test harness for abdrive library. 
 *
 * See learn.parallax.com/activitybot for more info.
 */

#include "simpletools.h" 
#include "arlodrive.h"

int main()                     
{
  //drive_pins(14, 15, 12, 13); 

  drive_goto(100, 100);

  drive_speed(0, 0);
  #ifdef interactive_development_mode
  drive_record(1);
  #endif
  pause(300);
  drive_speed(-20, -20);
  pause(1600);
  #ifdef interactive_development_mode
  drive_record(0);
  #endif
  pause(6400);
  #ifdef interactive_development_mode
  drive_record(1);
  #endif
  pause(1600);
  drive_speed(0, 0);
  #ifdef interactive_development_mode
  drive_record(0);
  #endif

  #ifdef interactive_development_mode
  drive_displayControlSystem(0, 2500/11);
  #endif

  while(1);
}

