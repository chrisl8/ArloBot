/********************************************************************
The code is used to calibrate the odometry of the Arlo board.
The encoders are plugged in the activity board and the DISPERCOUNT, TRACKWIDTH and DIAERROR should be calibrated.
Fwd() and Turn() functions are used to drive the robot to a certain distance or angle.
A typical cablibration method can be found in "Borenstein, J., & Feng, L. (1995). UMBmark: A benchmark test for measuring odometry errors in mobile robots. Ann Arbor, 1001, 48109-2110."
********************************************************************/

#include "simpletools.h"
#include "arlodrive.h"

// Define the encoders pins 
#define LEFT_A 3 
#define LEFT_B 2 
#define RIGHT_A 1 
#define RIGHT_B 0 

// Variables need to be calibrated
#define DISPERCOUNT 0.01309
#define TRACKWIDTH 0.38
#define DIAERROR 1.00 // diameter error (Dr/Dl) 

// Odometry variables
static volatile long int left_ticks = 0, right_ticks = 0;
static volatile long int left_ticks_old = 0, right_ticks_old = 0;
static volatile int last_left_A = 2; last_right_A = 2;
static double X = 0.0, Y = 0.0, Theta = 0.0;

void Encoder(void *par);
unsigned int stack[128]; 

// Motion control variables
static int task = 0;
static double squareSize = 1.0;
static double dist = 0.0, angle = 0.0;
static double startX = 0.0, startY= 0.0, startTh = 0.0;
static int speed = 30;

void Park()
{
  dhb10_com("GO 0 0\r");
}  

void Fwd(double d, int spd)
{
  char s[32]; // Hold strings converted for sending to DHB-10
  memset(s, 0, 32);
  startX = X;
  startY = Y;
  dist = d;
  speed = spd;
  
  while (sqrt((X-startX)*(X-startX)+(Y-startY)*(Y-startY))<dist)
  {
    sprint(s, "GO %d %d\r", speed, speed);
    dhb10_com(s);
    print("Left ticks: %d, Right ticks: %d\n", left_ticks, right_ticks);
    print("Robot pos, x=%f, y=%f, theta=%f\n",X,Y,Theta);     
  }
  print("Robot pos, x=%f, y=%f, theta=%f\n",X,Y,Theta);
  Park();
}

void Turn(double a, int spd)
{
  char s[32]; // Hold strings converted for sending to DHB-10
  memset(s, 0, 32);
  startTh = Theta;
  angle = a*PI/180.0;
  speed = spd;
  
  while ( fabs(Theta-startTh) < fabs(angle))
  {
    if (angle < 0)
    {
      sprint(s, "GO %d %d\r", speed, -speed);
    } 
    else 
    {
      sprint(s, "GO %d %d\r", -speed, speed);
    }             
    dhb10_com(s);
    print("Left ticks: %d, Right ticks: %d\n", left_ticks, right_ticks);
    print("Robot pos, x=%f, y=%f, theta=%f\n",X,Y,Theta);     
  }
  print("Robot pos, x=%f, y=%f, theta=%f\n",X,Y,Theta);
  Park();
}

int main()
{
  cogstart(Encoder, NULL, stack, sizeof(stack));
  
  int dt = CLKFREQ / 1000;
  int t = CNT;
  
  print("Program starts\n");  
  Park();
  pause(2000);
  Turn(90.0,30); 
  // pause(2000);
  // Fwd(1,30);
  pause(2000);
  Park();
}

void Encoder(void *par)
{
  while(1)
  {
    int left_A = input(LEFT_A);
    int left_B = input(LEFT_B);
    int right_A = input(RIGHT_A);
    int right_B = input(RIGHT_B);
    
    if (last_left_A == 0)  
    {  
        if (left_A == 1)  
        {  
            if (left_B == 0)  
            {  
                left_ticks++;  
            }            
            else   
            {  
                left_ticks--;  
            }            
        }          
    }   
    else if (last_left_A == 1) 
    { 
        if (left_A == 0)  
        {  
            if (left_B == 1)  
            {  
                left_ticks++;  
            }            
            else   
            {  
                left_ticks--;  
            }            
        }   
    }           
    last_left_A = left_A;  
      
    if (last_right_A == 0)  
    {  
        if (right_A == 1)  
        {  
            if (right_B == 0)  
            {  
                right_ticks++;  
            }            
            else   
            {  
                right_ticks--;  
            }            
        }          
    }   
    else if (last_right_A == 1)  
    {  
        if (right_A == 0)  
        {  
            if (right_B == 1)  
            {  
                right_ticks++;  
            }            
            else   
            {  
                right_ticks--;  
            }            
        }          
    }   
    last_right_A = right_A;    
    
    double deltaDistance, deltaTheta, deltaX, deltaY;
    int deltaTicksLeft, deltaTicksRight;
    
    deltaTicksLeft = left_ticks - left_ticks_old;
    deltaTicksRight = right_ticks - right_ticks_old;
    deltaDistance = 0.5f * (double) (deltaTicksLeft + deltaTicksRight * DIAERROR) * DISPERCOUNT;
    deltaTheta = (deltaTicksRight * DIAERROR - deltaTicksLeft) * DISPERCOUNT / TRACKWIDTH; 
    Theta += deltaTheta; 
    if (Theta > PI) 
      Theta -= 2 * PI; 
    if (Theta < -PI) 
      Theta += 2 * PI;
    deltaX = deltaDistance * cos(Theta);
    deltaY = deltaDistance * sin(Theta);       
    X += deltaX;
    Y += deltaY;
    
    left_ticks_old = left_ticks;
    right_ticks_old = right_ticks;
  }    
}  
