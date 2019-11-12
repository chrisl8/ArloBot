#ifndef ENCODER_COUNT_H
#define ENCODER_COUNT_H
/* This is ONLY used if the encoder pins
are connected directly to the Propeller board.
See #define encoderConnectedToPropeller
in per_robot_settings_for_propeller_c_code.h
*/

// Add encoders to the propeller board and start a cog to count the ticks
static volatile long int left_ticks = 0;
static volatile long int right_ticks = 0;

/**
 * For when the encoders are connected to the Propeller board
 * instead of being connected to the motor board.
 */
void encoderCount(void *par) {
  while (1) {
    int left_A = input(LEFT_A);
    int left_B = input(LEFT_B);
    int right_A = input(RIGHT_A);
    int right_B = input(RIGHT_B);

    if (last_left_A == 0) {
      if (left_A == 1) {
        if (left_B == 0) {
          left_ticks++;
        } else {
          left_ticks--;
        }
      }
    } else if (last_left_A == 1) {
      if (left_A == 0) {
        if (left_B == 1) {
          left_ticks++;
        } else {
          left_ticks--;
        }
      }
    }
    last_left_A = left_A;

    if (last_right_A == 0) {
      if (right_A == 1) {
        if (right_B == 0) {
          right_ticks++;
        } else {
          right_ticks--;
        }
      }
    } else if (last_right_A == 1) {
      if (right_A == 0) {
        if (right_B == 1) {
          right_ticks++;
        } else {
          right_ticks--;
        }
      }
    }
    last_right_A = right_A;
  }
}

#endif /* ENCODER_COUNT_H */
