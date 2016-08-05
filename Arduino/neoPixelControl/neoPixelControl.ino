#include <FastLED.h>
#define NUM_LEDS 119
// The last pixel on my strip is bad, it has no green.
#define DATA_PIN 6
 // This is where the top loop starts, for when I want to do things there and not on the part of the strip that leads up to it.
#define LOOP_START 33

// For FastLED:
CRGB leds [NUM_LEDS];

// For Serial string input:
#define COMMAND_LENGTH 8
int commandSegment = 0;
int commandArray[COMMAND_LENGTH];

void setup()                    // run once, when the sketch starts
{

    for (int i = 0; i < COMMAND_LENGTH; i++) {
      commandArray[commandSegment] = 0;
    }

  // Initialize FastLED
  FastLED.addLeds<NEOPIXEL, DATA_PIN>(leds, NUM_LEDS);

  // Set all to black on startup
  fill_solid(leds, NUM_LEDS, CRGB(0,0,0));
  FastLED.show();

  // Show a pretty rainbow on startup
//  rainbow(20, LOOP_START);

  Serial.begin(115200);           // set up Serial library at 9600 bps

  Serial.println("READY");

}

void loop()                       // run over and over again
{
  while (Serial.available()) {
    processInput();
    }
}

void handleCommand() {

  // print the string when a newline arrives:
    Serial.println("BUSY");
    switch (commandArray[0]) {
        case 0:
      colorWipe(commandArray[1], commandArray[2], commandArray[3], commandArray[4], commandArray[5], commandArray[6]);
      break;
      case 1:
      rainbow(commandArray[1], commandArray[2], commandArray[3]);
        break;
        case 2:

      rainbowCycle(commandArray[1], commandArray[2], commandArray[3]);
      break;
      case 3:
      theaterChase(commandArray[1], commandArray[2], commandArray[3], commandArray[4], commandArray[5],commandArray[6], commandArray[7]);
      break;
      case 4:
      theaterChaseRainbow(commandArray[1], commandArray[2], commandArray[3], commandArray[4]);
      break;
      case 9:
        fill_solid(leds, commandArray[4], CRGB(commandArray[1], commandArray[2], commandArray[3]));
        FastLED.show();
        break;
      default:
      Serial.println("UnknownCommand");
    }
    Serial.println("DONE");
}

/*
  SerialEvent occurs whenever a new data comes in the
 hardware serial RX.  This routine is run between each
 time loop() runs, so using delay inside loop can delay
 response.  Multiple bytes of data may be available.
 */
void processInput() {
    // get the new byte:
//    char inChar = (char)Serial.read();
    byte c = Serial.read();

    switch (c) {
      case '\n':
        handleCommand();
        for (int i = 0; i < COMMAND_LENGTH; i++) {
          commandArray[i] = 0;
        }
        commandSegment = 0;
        break;
      case ',':
        if (commandSegment < COMMAND_LENGTH) {
          commandSegment++;
        } else {
          Serial.println("OVERFLOW");
        }
        break;
      case '0' ... '9':
        commandArray[commandSegment] *= 10;
        commandArray[commandSegment] += c - '0';
        break;
    }
}

// NEOPIXEL FUNCTIONS FROM STRAND TEST
// CONVERTED TO FastLED
// Fill the dots one after the other with a color
int globalR, globalG, globalB;
void colorWipe(int r, int g, int b, int startingPixel, int endingPixel, int wait) {
  if (startingPixel > endingPixel) {
      for(int i=startingPixel; i >= endingPixel; i--) {
        leds[i].setRGB(r, g, b);
        FastLED.show();
        delay(wait);
      }
  } else {
      for(int i=startingPixel; i <= endingPixel; i++) {
        leds[i].setRGB(r, g, b);
        FastLED.show();
        delay(wait);
      }
  }
}

void rainbow(int startingPixel, int endingPixel, uint8_t wait) {
  uint16_t i, j;

  for(j=0; j<256; j++) {
    for(i=startingPixel; i< endingPixel; i++) {
      Wheel((i+j) & 255);
      leds[i].setRGB(globalR, globalG, globalB);
    }
    FastLED.show();
    delay(wait);
  }
}

// Slightly different, this makes the rainbow equally distributed throughout
void rainbowCycle(int startingPixel, int endingPixel, uint8_t wait) {
  uint16_t i, j;

  for(j=0; j<256*5; j++) { // 5 cycles of all colors on wheel
    for(i=LOOP_START; i< endingPixel; i++) {
      Wheel(((i * 256 / endingPixel) + j) & 255);
      leds[i].setRGB(globalR, globalG, globalB);
    }
    FastLED.show();
    delay(wait);
  }
}

//Theatre-style crawling lights.
void theaterChase(int r, int g, int b, uint8_t wait, int startingPixel, int endingPixel, int cycles) {
  for (int j=0; j<cycles; j++) {
    for (int q=0; q < 3; q++) {
      if (endingPixel > startingPixel) {
          //turn every third pixel on
          for (uint16_t i=startingPixel; i < endingPixel; i=i+3) {
            leds[i+q].setRGB(r, g, b);
          }
          FastLED.show();
          delay(wait);
          //turn every third pixel off
          for (uint16_t i=startingPixel; i < endingPixel; i=i+3) {
            leds[i+q].setRGB(0, 0, 0);
          }
        } else { // Reverse
          //turn every third pixel on
          for (uint16_t i=startingPixel; i > endingPixel; i=i-3) {
            leds[i-q].setRGB(r, g, b);
          }
          FastLED.show();
          delay(wait);
          //turn every third pixel off
          for (uint16_t i=startingPixel; i > endingPixel; i=i-3) {
            leds[i-q].setRGB(0, 0, 0);
          }
      }
    }
  }
}

//Theatre-style crawling lights with rainbow effect
void theaterChaseRainbow(uint8_t wait, int startingPixel, int endingPixel, int cycles) {
  for (int j=0; j < cycles; j++) {     // Try 256 to cycle all 256 colors in the wheel
    for (int q=0; q < 3; q++) {
      for (uint16_t i=startingPixel; i < endingPixel; i=i+3) {
        Wheel( (i+j) % 255);
        leds[i+q].setRGB(globalR, globalG, globalB);
//        strip.setPixelColor(i+q, Wheel( (i+j) % 255));    //turn every third pixel on
      }
      FastLED.show();

      delay(wait);

      for (uint16_t i=startingPixel; i < endingPixel; i=i+3) {
//        strip.setPixelColor(i+q, 0);        //turn every third pixel off
        leds[i+q].setRGB(0, 0, 0);
      }
    }
  }
}

// Input a value 0 to 255 to get a color value.
// The colours are a transition r - g - b - back to r.
void Wheel(byte WheelPos) {
  WheelPos = 255 - WheelPos;
  if(WheelPos < 85) {
    globalR = 255 - WheelPos * 3;
    globalG = 0;
    globalB = WheelPos * 3;
    return;
  }
  if(WheelPos < 170) {
    WheelPos -= 85;
    globalR = 0;
    globalG = WheelPos * 3;
    globalB = 255 - WheelPos * 3;
    return;
  }
  WheelPos -= 170;
  globalR = WheelPos * 3;
  globalG = 255 - WheelPos * 3;
  globalB = 0;
  return;
}
