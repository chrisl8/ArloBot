#ifndef POLL_PING_SENSORS_H
#define POLL_PING_SENSORS_H

void pollPingSensors(void *par);
static int pstack[128]; // If things get weird make this number bigger!

#ifdef hasMCP3208
#include "mcp3208_IR_cm.h"
#endif

void pollPingSensors(void *par) {
  int ir = 0, i;
  while (1) // Repeat indefinitely
  {
    for (i = 0; i < NUMBER_OF_PING_SENSORS; i++) {
      pingArray[i] = ping_cm(FIRST_PING_SENSOR_PIN + i);
#ifdef hasMCP3208
      // If there is also an IR sensor at this number check it too
      if (i < NUMBER_OF_IR_ON_MCP3208) {
        irArray[i] = mcp3208_IR_cm(i);
      }
#endif
    }
  }
}

#endif /* POLL_PING_SENSORS_H */
