/*
  Arlo-IR-Remote-Wav.c
  10/30/2013 version 0.5

  Arlo Robot plays WAV files on speaker from SD card
  Sensors: IR receiver on P4, PING))) distance sensor on P17
  Control with Sony-compatible remote (Parallax #020-00001)
  (Brightstar: hold Setup until lit, then enter 605)
  Drive with 5 buttons: Channel+/-, Vol +/-, and mute
  Number keys select WAV files to play
  If whiskers are pressed, robot backs up & stops, plays WAV
*/

#include "simpletools.h"
#include "wavplayer.h"                            // Needs 0.9 or later              
#include "sirc.h" 
#include "ping.h"             
#include "arlodrive.h"                            // Needs 0.5.5 or later

int key;                                          // Global var for remote key

int main()                                        // Main - execution begins!
{
  drive_speed(0,0);                               // Start servos/encoders cog
  drive_setRampStep(10);                          // Set ramping at 10 ticks/sec per 20 ms
  sirc_setTimeout(50);                            // Remote timeout = 50 ms

  // drive_feedback(0);

  int DO = 22, CLK = 23, DI = 24, CS = 25;        // Declare SD I/O pins
  sd_mount(DO, CLK, DI, CS);                      // Mount SD card
  wav_volume(7);                                  // Set vol here, 1 - 10  

  while(1)                                        // Outer loop
  {
    while(ping_cm(17) > 25)                       // Inner loop while no object within 10 cm 
    {
      int button = sirc_button(4);                // check for remote key press
  
      // Audio responses - if number key pressed, play named WAV file
      if(button == 1)wav_play("hello.wav");               
      if(button == 2)wav_play("follow.wav");   
      if(button == 3)wav_play("byebye.wav");                
      if(button == 4)wav_play("oops.wav");                
      if(button == 5)wav_play("thankyou.wav");                 
      if(button == 6)wav_play("dontknow.wav");                   
      if(button == 7)wav_play("yes.wav");               
      if(button == 8)wav_play("no.wav");                
      if(button == 9)wav_play("maybe.wav"); 
      if(button == 0)wav_play("electro.wav");                  
  
      // Motion responses - if key pressed, set wheel speeds
      if(button == CH_UP)
        drive_rampStep(100, 100);                 // Forward
      if(button == CH_DN)
        drive_rampStep(-100, -100);               // Backward
      if(button == VOL_DN)
        drive_rampStep(-100, 100);                // Left turn      
      if(button == VOL_UP)
        drive_rampStep(100, -100);                // Right turn 
      if(button == MUTE)
        drive_rampStep(0, 0);                     // Stop 
    }

    // Sensor response - PING))) sees object close by
    drive_speed(0, 0);                            // Stop driving
    pause(250);
    wav_play("excuseme.wav");                     // Play named WAV file
    drive_speed(-50, -50);
    pause(1500);
    drive_speed(0, 0);                            // Stop driving
  }            
}

