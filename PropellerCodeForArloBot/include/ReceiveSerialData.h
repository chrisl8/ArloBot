#include "CalculateChecksum.h"
#include <stdbool.h>

#define START_MARKER 254
#define END_MARKER 255
#define SPECIAL_BYTE 253
#define MAXIMUM_SERIAL_BUFFER_SIZE 252
// Currently a 253 ("special bit") in the length field indicates an error
// message
// We could add more "signal bits" if we want to cut the max lenght down more.

uint8_t receiveData(fdserial *term, uint8_t *pcData,
                    uint8_t pcDataInputLength) {
  // https://forum.arduino.cc/index.php?topic=288234.0
  // https://stackoverflow.com/questions/572547/what-does-static-mean-in-c
  // 1. A static variable inside a function keeps its value between invocations.
  // 2. A static global variable or a function is "seen" only in the file it's
  // declared in
  static bool receiveInProgress = false;
  static uint8_t receiveIndex = 0;
  static bool error = false;
  static uint8_t tempBuffer[MAXIMUM_SERIAL_BUFFER_SIZE];
  static uint8_t dataLength = 0;
  static uint8_t dataType = 0;
  uint8_t rc;
  uint8_t newData = 0;
  // This does two things:
  // 1. We don't polute the global struct until we have an entire packet that is
  // known good.
  // 2. Allows us to performa checksum on the data before we put it into the
  // struct

  // Keep looping if there is more data in a "packet",
  // but once a packet is found, don't overwrite it!
  while (fdserial_rxReady(term) > 0 && newData == 0) {
    rc = fdserial_rxChar(term);
    // NOTE: If this is getting stuck, use fdserial_rxTime with a timeout
    // instead, but since we checked with fdserial_rxReady taht shouldn't
    // happen.

    if (receiveInProgress) {
      if (rc != END_MARKER) {
        if (dataLength == 0 && !error) {
          // First byte should be data length
          dataLength = rc;
          if (dataLength < 1 || dataLength > MAXIMUM_SERIAL_BUFFER_SIZE) {
            dprint(term, "%c%ceInvalid data length: %d%c", START_MARKER,
                   SPECIAL_BYTE, dataLength, END_MARKER);
            error = true;
          }
        } else if (dataType == 0) {
          dataType = rc;
        } else {
          // Check for long packet
          // NOTE: Because we checked that dataLength is not longer than
          // MAXIMUM_SERIAL_BUFFER_SIZE already, this check suffices to make
          // sure we do not overrun tempBuffer
          if (receiveIndex < dataLength) {
            tempBuffer[receiveIndex] = rc;
            receiveIndex++;
          } else {
            if (!error) {
              // We have to add 1 to receiveIndex, because that is what the data
              // length WOULD be, if we added this bit, but we aren't adding it,
              // we are just discarding the entire packet until we hit an end
              // marker or another start marker.
              dprint(term, "%c%ceData length %d is longer than length bit %d%c",
                     START_MARKER, SPECIAL_BYTE, receiveIndex + 1, dataLength,
                     END_MARKER);
            }
            error = true;
            // We keep reading though, to clear all data until a valid closing
            // marker.
          }
        }
      } else {
        // End Marker received, validate, parse, and store data

        // Only use data if packet was not long ("Excess Data")
        if (!error) {
          // Check for packets less than given lenght (dataLength, which
          // includes High Byte encoding)
          if (receiveIndex != dataLength) {
            dprint(term, "%c%ceData length %d does not match length bit %d%c",
                   START_MARKER, SPECIAL_BYTE, receiveIndex, dataLength,
                   END_MARKER);
            error = true;
          }

          // Set dataLength based on data type
          if (!error) {
#include "SerialDataVariables.h"
            // Only list INCOMING packet types here.
            switch (dataType) {
            case testDataCharacter:
              pcDataInputLength = testDataLength;
              break;
            case initDataCharacter:
              pcDataInputLength = initDataLength;
              break;
            case settingsDataCharacter:
              pcDataInputLength = settingsDataLength;
              break;
            case moveDataCharacter:
              pcDataInputLength = moveDataLength;
              break;
            case ledDataCharacter:
              pcDataInputLength = ledDataLength;
              break;
            case positionUpdateDataCharacter:
              pcDataInputLength = positionUpdateDataLength;
              break;
            case abdOverrideDataCharacter:
              pcDataInputLength = abdOverrideDataLength;
              break;
            }
          }

          // decode High Bytes
          if (!error) {
            //  copies to dataRecvd[] only the data bytes i.e. excluding the
            //  marker bytes and the count byte and converts any bytes of 253
            //  etc into the intended numbers
            uint8_t dataRecvCount = 0;
            for (uint8_t n = 0; n < receiveIndex; n++) {
              if (!error) {
                uint8_t x = tempBuffer[n];
                if (x == SPECIAL_BYTE) {
                  // debugToPC("FoundSpecialByte");
                  n++;
                  x = x + tempBuffer[n];
                }
                pcData[dataRecvCount] = x;
                dataRecvCount++;
                if (dataRecvCount > pcDataInputLength) {
                  dprint(
                      term,
                      "%c%ceData received %d longer than expectd length %d%c",
                      START_MARKER, SPECIAL_BYTE, dataRecvCount,
                      pcDataInputLength, END_MARKER);
                  error = true;
                }
              }
            }
            // Check for packet shorter than expected
            if (!error && dataRecvCount < pcDataInputLength) {
              dprint(term,
                     "%c%ceData received %d shorter than expected length %d%c",
                     START_MARKER, SPECIAL_BYTE, dataRecvCount,
                     pcDataInputLength, END_MARKER);
              // Mark data as no good
              error = true;
            }
          }

          // If data was good so far, do a checksum
          if (!error) {
            uint8_t checkSumData =
                calculateChecksum(pcData, 0, pcDataInputLength - 1);
            if (checkSumData != pcData[pcDataInputLength - 1]) {
              error = true;
              dprint(term, "%c%ceChecksum %d does not match sent checksum %d%c",
                     START_MARKER, SPECIAL_BYTE, checkSumData,
                     pcData[pcDataInputLength - 1], END_MARKER);
            }
          }
          // When newData == 1 is returned, the struct(s) will be filled from
          // the pcData array
        }
        if (!error) {
          newData = dataType;
        }
        receiveInProgress = false;
        error = false;
      }
    }

    else if (rc == START_MARKER) {
      receiveInProgress = true;
      dataLength = 0;
      dataType = 0;
      error = false;
      receiveIndex = 0;
    } else {
      if (!error) {
        // Send error notice ONCE, not for every byte.
        dprint(term, "%c%ceData received outside of packet.%c", START_MARKER,
               SPECIAL_BYTE, END_MARKER);
      }
      error = true;
    }
  }
  return newData;
}
