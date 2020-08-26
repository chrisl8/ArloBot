
// =============================================== //
// * Send Data Over Serial *
// Any cog can use this to send data over serial
// It uses a semaphore to ensure two sends do not
// happen at the same time, and times out to avoid
// locking up or creating a huge backlog of data
// to send.
// =============================================== //

#ifndef SEND_OVER_SERIAL_H
#define SEND_OVER_SERIAL_H
#include "encodeHighBits.h"
#include <stdbool.h>

bool serialBusy = false;

// Timing out ensures we don't lock up on
// a busy line, or send data that is very
// old.
#define serialWriteTimeout 1000

// Making the delay too long severly slows down
// the serial send/receive speed on a busy
// connection.
#define serialContentionDelay 0.0001

// This is just used for the errror message.
// it helps with debugging and tweaking
// to know what the timeout is.
#define totalTimeoutDelay (serialWriteTimeout * serialContentionDelay)

void sendOverSerial(uint8_t *inputArray, uint8_t dataLength,
                    uint8_t dataCharacter) {
  uint8_t waiting = 0;
  while (serialBusy) {
    waiting++;
    if (waiting > serialWriteTimeout) {
      uint8_t delay = dprint(
          term, "%c%ceTimeout %f seconds elapsed trying to send over serial.%c",
          START_MARKER, SPECIAL_BYTE, totalTimeoutDelay, END_MARKER);
      return;
    }
    sleep(serialContentionDelay);
  }
  serialBusy = true;
  // Encode high bits and get data length
  uint8_t tempBuffer[MAXIMUM_SERIAL_BUFFER_SIZE];
  uint8_t outputByteCount =
      encodeHighBits(term, tempBuffer, inputArray, dataLength);

  if (outputByteCount > 0) {
    // Send Data
    fdserial_txChar(term, START_MARKER);
    fdserial_txChar(term, outputByteCount);
    fdserial_txChar(term, dataCharacter);
    for (uint8_t n = 0; n < outputByteCount; n++) {
      fdserial_txChar(term, tempBuffer[n]);
    }
    fdserial_txChar(term, END_MARKER);
  }
  serialBusy = false;
}

#endif /* SEND_OVER_SERIAL_H */
