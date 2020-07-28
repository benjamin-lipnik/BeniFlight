#ifndef CONFIGURATION_H
#define CONFIGURATION_H

/*COMMUNICATION, RADIO*/

//#define USE_IBUS
  //if ibus is enabled, serial printing will be unavailable

#ifndef USE_IBUS
  #define ENABLE_PRINTING
#endif


/*IMU*/


#define X_INDEX 0
#define Y_INDEX 1
#define Z_INDEX 2

#define ROLL_INDEX  X_INDEX
#define PITCH_INDEX Y_INDEX
#define YAW_INDEX   Z_INDEX

/*MOTORS*/

#define MOTOR_PIN_A   PB6
#define MOTOR_PIN_B   PB7
#define MOTOR_PIN_C   PB8
#define MOTOR_PIN_D   PB9

#define MOTOR_A_INDEX 0
#define MOTOR_B_INDEX 1
#define MOTOR_C_INDEX 2
#define MOTOR_D_INDEX 3


#endif
