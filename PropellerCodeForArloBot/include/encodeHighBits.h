#ifndef ENCODE_HIGH_BITS_H
#define ENCODE_HIGH_BITS_H
#include <stdbool.h>

uint8_t encodeHighBits(fdserial *term, uint8_t *tempBuffer, uint8_t *inputArray,
                       uint8_t end) {
  uint8_t outputByteCount = 0;
  for (uint8_t n = 0; n < end; n++) {
    if (inputArray[n] >= SPECIAL_BYTE) {
      tempBuffer[outputByteCount] = SPECIAL_BYTE;
      outputByteCount++;
      if (outputByteCount > MAXIMUM_SERIAL_BUFFER_SIZE) {
        dprint(term, "%c%ceAttempted to send more than %d bytes%c",
               START_MARKER, SPECIAL_BYTE, MAXIMUM_SERIAL_BUFFER_SIZE,
               END_MARKER);
        outputByteCount = 0;
        break;
      }
      tempBuffer[outputByteCount] = inputArray[n] - SPECIAL_BYTE;
    } else {
      tempBuffer[outputByteCount] = inputArray[n];
    }
    outputByteCount++;
    if (outputByteCount > MAXIMUM_SERIAL_BUFFER_SIZE) {
      dprint(term, "%c%ceAttempted to send more than %d bytes%c", START_MARKER,
             SPECIAL_BYTE, MAXIMUM_SERIAL_BUFFER_SIZE, END_MARKER);
      outputByteCount = 0;
      break;
    }
  }
  return outputByteCount;
}

#endif /* ENCODE_HIGH_BITS_H */
