//#define interactive_development_mode

/*
  abdrive.c library source
*/


#include "arlodrive.h"                   // Include servo lib funct defs
#include "simpletools.h"
#include "simpletext.h"
#include "fdserial.h"
#include <string.h>

void drive_com(int arrayLcnt, int arrayRcnt, 
               int centerL, int centerR, 
               int* pwAddrL, int* pwAddrR, 
               int* spdAddrL, int* spdAddrR);
void drive_set(int left, int right);
void encoders(void *par);
void interpolate(int* ltmp, int* rtmp);
void interpolation_table_setup();
void servos_diffDrive(void);
void drive_record(int startStop);
void drive_displayControlSystem(int start, int end);
void set_drive_speed(int left, int right);


static volatile int abd_pLo = 12;           // Left offset
static volatile int abd_pLd = 12;          // Left divisor
static volatile int abd_iLinc = 1;         // Left integral increment

static volatile int abd_pRo = 12;           // Right offset
static volatile int abd_pRd = 12;          // Right divisor
static volatile int abd_iRinc = 1;         // Right integral increment

// drive_trimset
volatile int abd_trimFL, abd_trimFR, abd_trimBL, abd_trimBR, abd_trimticksF, abd_trimticksB;
volatile int abd_trimticksF = 0;
volatile int abd_trimticksB = 0;
int abd_eeAddr;

// drive_goto
volatile int abd_ticksL = 0;
volatile int abd_ticksR = 0;
volatile int abd_speedL;      // Requested servo speed left
volatile int abd_speedR;      // Requested servo speed right
int abd_rampStep = 4;
int abd_speedLimit = 100;
volatile int abd_zeroDelay = ON;
volatile int abd_dlc;      // distance left calculated
volatile int abd_drc;      // distance right calculated
volatile int abd_dlca = 0;                                 // distance left calculated (accumulated)
volatile int abd_drca = 0;                                 // distance right calculated (accumulated)
volatile int abd_dsr = 400;
volatile int abd_edL;                                      // error distance left
volatile int abd_edR;                                      // error distance right
volatile int abd_pL;                                       // proportional left
volatile int abd_pR;                                       // proportional right
volatile int abd_iL;                                   // integral left
volatile int abd_iR;                                   // integral right
volatile int abd_eaL = 0;
volatile int abd_eaR = 0;
volatile unsigned int _servoPulseReps;

// servoPins
volatile int abd_sPinL = 12, abd_sPinR = 13;   // Global variables
volatile int abd_ePinL = 14, abd_ePinR = 15;
volatile int abd_us;
volatile int abd_intTabSetup = 0;

// debug
volatile int abd_record = 0;               // Record values to an array

// display
volatile int abd_elCntL;
volatile int abd_elCntR;
volatile int abd_cntrLidx;
volatile int abd_cntrRidx;
int abd_spdrL[120];
int abd_spdmL[120];
int abd_spdrR[120];
int abd_spdmR[120];

static volatile int cntrLval;
static volatile int cntrRval;

static int cog = 0;
//static int servoCog2 = 0;
static unsigned int stack[(160 + (125 * 4)) / 4];
//static unsigned int servoStack[(160 + (150 * 4)) / 4];


//static int a = 0;
static int r = 0;

//static int intTabSetup = 0;
//static int eeStarted = 0;

static volatile int trimctr = 0;
static volatile int dca, trimticks;

static volatile int leftPrev, rightPrev;

static volatile int kp[6];

static volatile int tcL;
static volatile int tcR;
static volatile int tiL;


volatile int abd_ridx = 0;


static volatile int tiR;

static volatile int stateL;
static volatile int stateR;

static volatile int* pwL;
static volatile int* pwR;
static volatile int* spdL;
static volatile int* spdR;

static volatile int etpsR;    // encoder ticks per second requested right
static volatile int etpsL;    // encoder ticks per second requested right

static volatile int pcount;
static volatile unsigned int _sprOld;
static volatile unsigned int _sprNext;


static volatile int ssiL; // servo speeed interpolated left
static volatile int ssiR; // servo speeed interpolated right

static volatile int driveL;
static volatile int driveR;

static volatile int phsL;
static volatile int phsR;
static volatile int phsLr;
static volatile int phsRr;

static int trimFunction = 1;
static int encoderFeedback = 1;

static int speedLprev = 0, speedRprev = 0;

volatile int xbee_setup = 0;

#ifdef interactive_development_mode
volatile int abd_rv[2600];
#endif // interactive_development_mode



int drive_open()
{
  if(!abd_intTabSetup)
  {
    interpolation_table_setup();
    set_drive_speed(0, 0);
  }
  return cog;
}

void drive_close()
{
  if(cog)
  {
    cogstop(cog - 1);
    cog = 0;
  }
}

void drive_setMaxSpeed(int maxTicksPerSec)
{
  abd_speedLimit = maxTicksPerSec;
}


void drive_setRampStep(int stepsize)
{
  abd_rampStep = stepsize;
}


void drive_feedback(int enabled)
{
  encoderFeedback = enabled;
}


void drive_trim(int enabled)
{
  trimFunction = enabled;
}


void drive_com(int arrayLcnt, int arrayRcnt, 
               int centerL, int centerR, 
               int* pwAddrL, int* pwAddrR, 
               int* spdAddrL, int* spdAddrR)
{
  abd_elCntL = arrayLcnt;
  abd_elCntR = arrayRcnt;
  abd_cntrLidx = centerL;
  abd_cntrRidx = centerR;
  pwL = pwAddrL;
  pwR = pwAddrR;
  spdL = spdAddrL;
  spdR = spdAddrR;
  cntrLval = pwAddrL[abd_cntrLidx];
  cntrRval = pwAddrR[abd_cntrRidx];
}


void interpolation_table_setup()
{
  if(!abd_us) abd_us = CLKFREQ/1000000; 

  //ee_putStr("ActivjtyBot", 12, _ActivityBot_EE_Start_);
  //putStr("hello");

  unsigned char str[12];
  ee_getStr(str, 12, _ActivityBot_EE_Start_);
  ee_getStr(str, 12, _ActivityBot_EE_Start_);

  /*
  if(strcmp(str, "ActivityBot"))
  {
    putStr("Calibrate your ActivityBot first!\n");
    putStr("For info, go to learn.parallax.com/ActivityBot/Calibrate-Your-ActivityBot\n");
    //drive_feedback(0);
  }
  */

  abd_eeAddr = _ActivityBot_EE_Start_ + _ActivityBot_EE_Left_;
  //print("left abd_eeAddr = %d\n", abd_eeAddr);
  int cntL = ee_getInt(abd_eeAddr);
  abd_eeAddr += 4;
  int zstartL = ee_getInt(abd_eeAddr);
  abd_eeAddr += 4;
  for(r = 0; r < cntL; r++)
  {
    abd_spdrL[r] = ee_getInt(abd_eeAddr);
    abd_eeAddr+=4;
    abd_spdmL[r] = ee_getInt(abd_eeAddr);
    abd_eeAddr += 4;  
  }
  abd_spdmL[cntL - 1] = 1000;
  abd_spdmL[0] = 1000;

  abd_eeAddr = _ActivityBot_EE_Start_ + _ActivityBot_EE_Right_;
  //print("right abd_eeAddr = %d\n", abd_eeAddr);
  int cntR = ee_getInt(abd_eeAddr);
  abd_eeAddr += 4;
  int zstartR = ee_getInt(abd_eeAddr);
  abd_eeAddr += 4;
  for(r = 0; r < cntR; r++)
  {
    abd_spdrR[r] = ee_getInt(abd_eeAddr);
    abd_eeAddr+=4;
    abd_spdmR[r] = ee_getInt(abd_eeAddr);
    abd_eeAddr += 4;  
  } 
  abd_spdmR[cntR - 1] = 1000;
  abd_spdmR[0] = 1000;

  drive_com(cntL, cntR, zstartL, zstartR, abd_spdrL, abd_spdrR, abd_spdmL, abd_spdmR);

  abd_eeAddr      =  _ActivityBot_EE_Start_ + _ActivityBot_EE_Trims_;
  //print("trims abd_eeAddr = %d\n", abd_eeAddr);
  abd_trimFL      =  ee_getInt(abd_eeAddr +  0);
  abd_trimFR      =  ee_getInt(abd_eeAddr +  4);
  abd_trimBL      =  ee_getInt(abd_eeAddr +  8);
  abd_trimBR      =  ee_getInt(abd_eeAddr + 12);
  abd_trimticksF  =  ee_getInt(abd_eeAddr + 16);
  abd_trimticksB  =  ee_getInt(abd_eeAddr + 20);

  int eeAddr = _ActivityBot_EE_Start_  + _ActivityBot_EE_Pins_;
  unsigned char pinInfo[16];

  for(int i = 0; i < 16; i++) 
    pinInfo[i] = ee_getByte(eeAddr + i);

  if(pinInfo[0] == 's' && pinInfo[1] == 'p' && pinInfo[2] == 'L' && pinInfo[5] == 'R')
  {
    abd_sPinL = (int) pinInfo[3];
    abd_sPinR = (int) pinInfo[6];
  }
    
  if(pinInfo[8] == 'e' && pinInfo[9] == 'p' && pinInfo[10] == 'L' && pinInfo[13] == 'R')
  {
    abd_ePinL = (int) pinInfo[11];
    abd_ePinR = (int) pinInfo[14];
  }

  //print("abd_spinL = %d, abd_sPinR = %d, abd_epinL = %d, abd_ePinR = %d\n", 
  //       abd_sPinL,      abd_sPinR,      abd_ePinL,      abd_ePinR); 

  abd_intTabSetup = 1;
}


void interpolate(int *ltmp, int *rtmp)
{
  
  int left = *ltmp;
  int right = *rtmp;

  /////printf("\netpsL = %d, etpsR = %d\n\n", etpsL, etpsR);

  int listep;
  int limit;
  int lookupval;

  if(left > 0)
  {
    listep = 1;
    limit = abd_elCntL;
    lookupval = left;
  }
  else
  {
    listep = -1;
    limit = 0;
    lookupval = -left;
  }

  int rprev = abd_cntrLidx;

  for(int r = abd_cntrLidx; r != limit; r+=listep)
  {
    if(spdL[r] == lookupval)
    {
      left = pwL[r]; 
      break;
    }
    if((spdL[rprev] < lookupval) && (spdL[r] > lookupval))
    {
      int x = ((pwL[r]-pwL[rprev])*(lookupval-spdL[rprev]))/(spdL[r]-spdL[rprev]); 
      left = pwL[rprev] + x; 
      break;
    }
    rprev = r;
  }
  //if(r >= elCntL) left = pwL[elCntL];                    // 2013.08.17
  //if(r <= 0) left = pwL[0];                              // 2013.08.17
  //if(r >= elCntL) left = *ltmp;                    // 2013.08.17
  //if(r <= 0) left = *ltmp;                              // 2013.08.17

  if(right > 0)
  {
    listep = 1;
    limit = abd_elCntL;
    lookupval = right;
  }
  else
  {
    listep = -1;
    limit = 0;
    lookupval = -right;
  }

  rprev = abd_cntrRidx;

  for(int r = abd_cntrRidx; r != limit; r+=listep)
  {
    if(spdR[r] == lookupval)
    {
      right = pwR[r]; 
      break;
    }
    if((spdR[rprev] < lookupval) && (spdR[r] > lookupval))
    {
      int x = ((pwR[r]-pwR[rprev])*(lookupval-spdR[rprev]))/(spdR[r]-spdR[rprev]); 
      right = pwR[rprev] + x; 
      break;
    }
    rprev = r;
  }
  //if(r >= elCntR) right = pwR[elCntR];                    // 2013.08.17
  //if(r <= 0) right = pwR[0];                              // 2013.08.17
  //if(r >= elCntR) right = *rtmp;                          // 2013.08.17
  //if(r <= 0) right = *rtmp;                               // 2013.08.17

  *ltmp = left;
  *rtmp = right;
}


void set_drive_speed(int left, int right)
{
  
  if(encoderFeedback)
  {
    if(left > abd_speedLimit) left = abd_speedLimit;
    if(left < -abd_speedLimit) left = -abd_speedLimit;
    if(right > abd_speedLimit) right = abd_speedLimit;
    if(right < -abd_speedLimit) right = -abd_speedLimit;
  }

  int leftTemp = left;
  int rightTemp = right;

  interpolate(&leftTemp, &rightTemp);

  etpsL = left;
  etpsR = right;

  ssiL = leftTemp;
  ssiR = -rightTemp;

  abd_speedL = left;
  abd_speedR = right;

  if(!cog)
  {
    /////printf("\n\n!!!!! Starting COG !!!!!!\n\n");
    cog = 1 + cogstart(&encoders, NULL, stack, sizeof(stack)-1);
  }  
}


void drive_speed(int left, int right)        // driveSpeeds function
{
  if(!abd_intTabSetup)
  {
    interpolation_table_setup();
    set_drive_speed(0, 0);
    //pause(40);
  }
  
  //
  if(abd_zeroDelay == ON)
  {
    if((speedLprev > 0 && left <= 0) || (speedLprev < 0 && left >= 0) || (speedRprev > 0 && right <= 0) || (speedRprev < 0 && right >= 0))
    {
      int tempLeftZ = left;
      int tempRightZ = right;
      if((speedLprev > 0 && left <= 0) || (speedLprev < 0 && left >= 0))
      {
        tempLeftZ = 0;
      } 
      if((speedRprev > 0 && right <= 0) || (speedRprev < 0 && right >= 0))
      {
        tempRightZ = 0;
      } 
      set_drive_speed(tempLeftZ, tempRightZ);
      speedLprev = tempLeftZ;
      speedRprev = tempRightZ;
      pause(120); 
    }
  }

  //_sprNext = _servoPulseReps;
  //while((_sprNext+1) >= _servoPulseReps);

  set_drive_speed(left, right);

  speedLprev = abd_speedL;
  speedRprev = abd_speedR;
}


void encoders(void *par)
{

  _servoPulseReps = 0;
  //int oneshot = 0;

  OUTA &= ~(1 << abd_sPinL); 
  OUTA &= ~(1 << abd_sPinR); 
  DIRA |= 1 << abd_sPinL; 
  DIRA |= 1 << abd_sPinR; 

  pause(20);
  //pause(1);

  int tempL = 0;
  int tempR = 0;
  interpolate(&tempL, &tempR);  

  PHSA = 0;
  PHSB = 0;
  FRQA = 1;
  FRQB = 1;
  CTRA  = abd_sPinL | (4 << 26);
  CTRB  = abd_sPinR | (4 << 26);
   
  phsL = (1500 + tempL);
  phsR = (1500 - tempR);
  phsLr = phsL;
  phsRr = phsR;

  int t = CNT;
  int dt1 = 13*(CLKFREQ/1000);
  int dt2 = 7*(CLKFREQ/1000);

  phsL = phsLr;
  phsR = phsRr;
  PHSA = -phsL*abd_us;
  PHSB = -phsR*abd_us;
  //waitcnt(t+=dt1);
  //waitcnt(t+=dt);
  _servoPulseReps++;
  //waitcnt(t+=dt2);
  //while(1);
  //t+=(dt1+dt2);
  low(12);
  low(13);

  pause(20);
  //pause(1);
  PHSA = -phsL*abd_us;
  PHSB = -phsR*abd_us;

  t+=(dt1+dt2);


  int inc = 0;
  int diff = 0;
  int trimAccum = 0;

  int zdirL = 0;
  int zdirR = 0;

  stateL = (INA >> abd_ePinL) & 1;
  stateR = (INA >> abd_ePinR) & 1;

  while(!_servoPulseReps);

  //int dsrL = 400;                                // distance sample rate
  //int dsrR = 394;                                // distance sample rate
  int tdst = CLKFREQ/abd_dsr;                       // time of distance sample
  int td = CNT + tdst;                          // time of distance
  //int tdsn = 0;                                 // time of distance sample number
  _sprOld = _servoPulseReps;

  abd_edL = 0;                                      // error distance left
  abd_edR = 0;                                      // error distance right

  abd_pL = 0;                                       // proportional left
  abd_pR = 0;                                       // proportional right

  abd_iL = 0;                                   // integral left
  abd_iR = 0;                                   // integral right

  int maxIR = 0;
  int maxIL = 0;

  while(1)
  {
    // Left encoder
    if(((INA >> abd_ePinL) & 1) != stateL)
    {
      stateL = (~stateL) & 1;
      if(stateL == 1) 
      {
        if((CNT - tiL) > (CLKFREQ/400))
        {
          tiL = CNT;
        }  
      }
      if(phsL > cntrLval + 1500)
      {
        abd_ticksL++;
        zdirL = 1;
      }
      else if(phsL < cntrLval + 1500)
      {
        abd_ticksL--;
        zdirL = -1;
      }
      else
      {
        abd_ticksL += zdirL;
      }
    }
    
    // Right encoder
    if(((INA >> abd_ePinR) & 1) != stateR)
    {
      stateR = (~stateR) & 1;
      if(stateR == 1) 
      {
        if((CNT - tiR) > (CLKFREQ/400))
        {
          tiR = CNT;
        }  
      }
      if(phsR < 1500 - cntrRval)
      {
        abd_ticksR++;
        zdirR = 1;
      }
      else if(phsR > 1500 - cntrRval)
      {
        abd_ticksR--;
        zdirR = -1;
      }
      else
      {
        abd_ticksR += zdirR;
      }
    }

    // Calculated distance accumulator
    if((td - CNT) > tdst) 
    {
  
      td += tdst;                                 // Reset sample timer         '

      abd_dlca += etpsL;                              // + velocityL for dt
      abd_drca += etpsR;                              // + velocityR for dt
      
      //dlca -= 70*(edL - edR);
      
      abd_dlc = abd_dlca/abd_dsr;                             // ticks expected
      abd_drc = abd_drca/abd_dsr;                             

      //#define test_trim_settings_new
      //#ifdef test_trim_settings_new
      if(trimFunction)
      {
        if((((abd_speedL > 0)&&abd_trimFL))||((abd_speedR > 0)&&(abd_trimFR)))
        {
          trimticks = abd_trimticksF;
          dca = abd_dlca*abd_trimFL + abd_drca*abd_trimFR;
        }
        else if((((abd_speedL < 0)&&abd_trimBL))||((abd_speedR < 0)&&(abd_trimBR)))
        {
          trimticks = abd_trimticksB;
          dca = abd_dlca*abd_trimBL + abd_drca*abd_trimBR;
        }
        if((((abd_speedL > 0)&&abd_trimFL))||((abd_speedR > 0)&&(abd_trimFR)))
        {
          if(dca >= trimAccum)
          {
            diff = dca - trimAccum;
            inc = diff/trimticks;
            dca += inc;
            trimAccum += inc;
            trimAccum += (inc*trimticks);         
          }
          abd_dlca += (abd_trimFL*inc);
          abd_drca += (abd_trimFR*inc);
        }
        else if((((abd_speedL < 0)&&abd_trimBL))||((abd_speedR < 0)&&(abd_trimBR)))
        {
          if(dca <= trimAccum)
          {
            diff = dca - trimAccum;
            inc = diff/trimticks;
            dca += inc;
            trimAccum += inc;
            trimAccum += (inc*trimticks);         
          }
          abd_dlca += (abd_trimBL*inc);
          abd_drca += (abd_trimBR*inc);
        }
      }
      //#endif // test_trim_settings_new3
    }
      
    //#define no_control 

    // wait until 15 ms into servo control cycle
    //if(_servoPulseReps != _sprOld)

    /*
    if((CNT - t) >= dt1 && (oneshot == 0))
    {
      oneshot = 1;
      _servoPulseReps++;
    }
    */

    if((CNT - t) >= (dt1 + dt2))
    {
      t+=(dt1+dt2);
      //oneshot = 0;
      //pulseTime = CNT;
      _sprOld = _servoPulseReps;
      pcount++;
      
      // Distance controller
      // #if 1
      if(encoderFeedback)
      {
        abd_edL = abd_dlc - abd_ticksL;
        abd_eaL += abd_edL;
        if(abd_speedL != 0)
        {
          //iL += edL;
          if(abd_speedL > 0)
          {
            abd_pL = abd_edL * (abd_pLo+(abd_speedL/abd_pLd));  
            if(abd_edL>0)abd_iL+=abd_iLinc; else if(abd_edL<0) abd_iL-=abd_iLinc;
          }
          else if(abd_speedL < 0)
          {
            abd_pL = abd_edL * (-abd_pLo+(abd_speedL/abd_pLd));  
            if(abd_edL>0)abd_iL-=abd_iLinc; else if(abd_edL<0) abd_iL+=abd_iLinc;
          }
          maxIL = abd_speedL;
          if(maxIL < 0) maxIL = -maxIL;
          if(abd_iL > maxIL) abd_iL = maxIL;
          if(abd_iL < -maxIL) abd_iL = -maxIL;
          //iL = 0;
          if(abd_speedL > 0)
            driveL = abd_iL + abd_pL + ssiL + 1500;
            //if(driveL < cntrLval + 1500) driveL = cntrLval + 1500;   
          if(abd_speedL < 0)
            driveL = -abd_iL - abd_pL + ssiL + 1500;
            //if(driveL > cntrLval + 1500) driveL = cntrLval + 1500;   
        }
        else
        {
          driveL = ssiL + 1500;
          abd_iL = 0;
        }

        abd_edR = abd_drc - abd_ticksR;
        abd_eaR += abd_edR;
        if(abd_speedR != 0)
        {
          //iR += edR;
          if(abd_speedR > 0)
          {
            abd_pR = abd_edR * (abd_pRo+(abd_speedR/abd_pRd));  
            if(abd_edR>0)abd_iR+=abd_iRinc; else if(abd_edR<0) abd_iR-=abd_iRinc;
          }
          else if(abd_speedR < 0)
          {
            abd_pR = abd_edR * (-abd_pRo+(abd_speedR/abd_pRd));  
            if(abd_edR>0)abd_iR-=abd_iRinc; else if(abd_edR<0) abd_iR+=abd_iRinc;
          }
          maxIR = abd_speedR;
          if(maxIR < 0) maxIR = - maxIR;
          if(abd_iR > maxIR) abd_iR = maxIR;
          if(abd_iR < -maxIR) abd_iR = -maxIR;
          //iR = 0;
          if(abd_speedR > 0)
            driveR = -abd_iR - abd_pR + ssiR + 1500;
            //if(driveR > 1500 - cntrRval) driveR = 1500 - cntrRval;
          if(abd_speedR < 0)
            driveR = abd_iR + abd_pR + ssiR + 1500;
            //if(driveR < 1500 - cntrRval) driveR = 1500 - cntrRval;
        }
        else
        {
          driveR = ssiR + 1500;
          abd_iR = 0;
        }
        //#if 1
        //if(encoderFeedback)
        //{
        phsLr = driveL; 
        phsRr = driveR; 
        //}
        //#endif
        // #endif
      }
      else
      {
        phsLr = ssiL + 1500;
        phsRr = ssiR + 1500;
      }

      phsL = phsLr;
      phsR = phsRr;
      PHSA = -phsL*abd_us;
      PHSB = -phsR*abd_us;
      _servoPulseReps++;


      //if(record)
      #ifdef interactive_development_mode
      if(abd_record)
      {
        //rv[ridx] = dlc;
        abd_rv[abd_ridx+0] = abd_speedL;
        abd_rv[abd_ridx+1] = abd_dlc;
        abd_rv[abd_ridx+2] = abd_ticksL;
        abd_rv[abd_ridx+3] = abd_edL;
        abd_rv[abd_ridx+4] = abd_pL;
        abd_rv[abd_ridx+5] = abd_iL;
        //abd_rv[ridx+5] = phsLr;
        abd_rv[abd_ridx+6] = abd_drc;
        abd_rv[abd_ridx+7] = abd_ticksR;
        abd_rv[abd_ridx+8] = abd_edR;
        abd_rv[abd_ridx+9] = abd_pR;
        abd_rv[abd_ridx+10] = abd_iR;
        //rv[ridx+10] = phsRr;
        abd_ridx += 11;
      }
      #endif // interactive_development_mode
    }
  }
}


