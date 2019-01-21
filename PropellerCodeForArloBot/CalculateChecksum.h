#ifndef CALCULATE_CHECKSUM_H
#define CALCULATE_CHECKSUM_H

uint8_t calculateChecksum(uint8_t *inputArray, uint8_t start, uint8_t end) {
  // https://henryforceblog.wordpress.com/2015/03/12/designing-a-communication-protocol-using-arduinos-serial-library/
  uint8_t checkSumData = 0;
  uint16_t sum = 0;
  for (uint8_t n = start; n < end; n++) {
    sum += inputArray[n];
  }
  checkSumData = sum & 0xFF;
  return checkSumData;
}

#endif /* CALCULATE_CHECKSUM_H */
