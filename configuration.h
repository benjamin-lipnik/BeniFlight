#ifndef CONFIGURATION_H
#define CONFIGURATION_H

/*COMMUNICATION, RADIO*/

#define USE_IBUS
  //if ibus is enabled, serial printing will be unavailable

#define ENABLE_PRINTING
#define ENABLE_PRINTING_AND_IBUS

#if (defined USE_IBUS) && (!defined ENABLE_PRINTING_AND_IBUS)
  #undef ENABLE_PRINTING
#endif

/*IMU*/

//#define GYRO_INVERT_X
#define GYRO_INVERT_Y
#define GYRO_INVERT_Z

#define ACC_INVERT_X
//#define ACC_INVERT_Y
//#define ACC_INVERT_Z

//#define MAG_INVERT_X
//#define MAG_INVERT_Y
//#define MAG_INVERT_Z


/*MOTORS*/

#define MOTOR_PIN_A   PB6
#define MOTOR_PIN_B   PB7
#define MOTOR_PIN_C   PB8
#define MOTOR_PIN_D   PB9

/*LED, BAT PINS*/

#define BLUE_LED_PIN  PB12
#define RED_LED_PIN   PB13
#define MCU_LED_PIN   PC13
#define BAT_SENSE_PIN PA0

/*BATTERY*/
//#define BAT_CELL_COUNT        3
#define CELL_VOLTAGE_LOW       (3.7f)
#define CELL_VOLTAGE_FULL      (4.2f)
#define CELL_VOLTAGE_CRITICAL  (3.4f)

/*STABILIZATION*/
//#define PROPS_OUT
#define STEER_MAX         100
#define MAX_USER_POWER    0.85f
#define IDLE_POWER        1100
#define LOST_SIGNAL_POWER 1300
#define ENABLE_AUTOLEVEL
//#define ENABLE_HEADLOCK
//#define ENABLE_HEADLESS

#ifdef ENABLE_AUTOLEVEL
  #define AUTOLEVEL_STRENGTH 3.0f
  #define ENABLE_FAILSAFE
  #define LOST_SIGNAL_FAILSAFE_TIMEOUT 10000
#endif

#ifdef ENABLE_HEADLOCK
  //#undef ENABLE_HEADLESS
  #define HEADLOCK_STRENGTH 1.5f
#endif

#ifdef ENABLE_HEADLESS
  //#undef ENABLE_HEADLOCK
#endif



/*GLOBAL CONSTANTS, IGNORE & DO NOT CHANGE!*/
#define INIT_OK    1
#define INIT_ERROR 0

#define ARM_BIT 0
#define FEATURE_1_BIT 1
#define FEATURE_2_BIT 2
#define FEATURE_3_BIT 3

//meh
#define FEATURE_1(instance) (instance.buttons & 1)
#define FEATURE_2(instance) (instance.buttons & 2)
#define FEATURE_3(instance) (instance.buttons & 4)
#define FEATURE_4(instance) (instance.buttons & 8)


#define X_INDEX 0
#define Y_INDEX 1
#define Z_INDEX 2

#define ROLL_INDEX  X_INDEX
#define PITCH_INDEX Y_INDEX
#define YAW_INDEX   Z_INDEX

#define MOTOR_A_INDEX 0
#define MOTOR_B_INDEX 1
#define MOTOR_C_INDEX 2
#define MOTOR_D_INDEX 3



#endif
