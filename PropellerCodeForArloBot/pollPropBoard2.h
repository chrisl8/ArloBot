#ifndef POLL_PROP_BOARDS_2_H
#define POLL_PROP_BOARDS_2_H

fdserial *propterm;
void pollPropBoard2(void *par);
static int prop2stack[256]; // If things get weird make this number bigger!

void pollPropBoard2(void *par) {
  const char delimiter[2] = ","; // Delimiter character for incoming messages
                                 // from the ROS Python script
  propterm = fdserial_open(QUICKSTART_RX_PIN, QUICKSTART_TX_PIN, 0, 115200);

#define bufferLength 10 // Longer than longest possible received line

// This is the incoming rate limiter. Without
// some limit the entire Propeller will hang.
#define rateLimit 5

  bool sendLEDs = false;
  while (1) {
    pause(rateLimit);
    // Tell the other end we are alive, so it doesn't just spin pointlessly.
    // It also keeps the sensors quiet when this end is in an idle state.
    if (sendLEDs) {
      sendLEDs = false;
      dprint(propterm, "l");
      for (uint8_t i = 0; i < NUMBER_OF_LEDS; i++) {
        dprint(propterm, "%d", ledArray[i]);
        pause(rateLimit);
      }
    } else {
      dprint(propterm, "i");
    }
    // Do not put any delay here.
    if (fdserial_rxReady(propterm) != 0) {
      // high(26); // LEDs for debugging
      int count = 0;
      char buf[bufferLength];
      while (count < bufferLength) {
        buf[count] = readChar(propterm);
        if (buf[count] == '.') // Using . for end of line instead of line break
          break;
        count++;
      }

      if (buf[0] == 'p') {
        char *token;
        token = strtok(buf, delimiter);
        token = strtok(NULL, delimiter);
        char *unconverted;
        uint8_t pingSensorNumber = strtol(token, &unconverted, 10);
        token = strtok(NULL, delimiter);
        if (pingSensorNumber < NUMBER_OF_PING_SENSORS) {
          pingArray[pingSensorNumber] = strtol(token, &unconverted, 10);
        }
      } else if (buf[0] == 'i') {
        char *token;
        token = strtok(buf, delimiter);
        token = strtok(NULL, delimiter);
        char *unconverted;
        uint8_t irSensorNumber = strtol(token, &unconverted, 10);
        token = strtok(NULL, delimiter);
        if (irSensorNumber < NUMBER_OF_IR_SENSORS) {
          irArray[irSensorNumber] = strtol(token, &unconverted, 10);
        }
      } else if (buf[0] == 'b') {
        char *token;
        token = strtok(buf, delimiter);
        token = strtok(NULL, delimiter);
        char *unconverted;
        uint8_t buttonNumber = strtol(token, &unconverted, 10);
        if (buttonNumber < NUMBER_OF_BUTTON_SENSORS) {
          buttonArray[buttonNumber] = 1;
        }
      } else if (buf[0] == 'l') {
        sendLEDs = true;
      }
      // low(26); // LEDs for debugging
    }
#ifdef hasFloorObstacleSensors
    for (uint8_t i = 0; i < NUMBER_OF_FLOOR_SENSORS; i++) {
      floorArray[i] = input(FIRST_FLOOR_SENSOR_PIN + i);
    }
#endif
  }
}

#endif /* POLL_PROP_BOARDS_2_H */
