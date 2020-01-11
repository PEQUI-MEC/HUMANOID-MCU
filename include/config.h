#ifndef CONFIG_H_
#define CONFIG_H_

#include <Arduino.h>

// TODO: Finish documentation on config
// TODO: Add documentation on extras/pid_tuning
// TODO: Add documentation on DMASerial

/**
 * Timing Definitions
 *
 * Defines delays and periods used throughout the program.
 */

// Delay before the main execution begins (ms).
#define INITIAL_DELAY 3000
// Delay between each ros-serial spin (ms).
#define SPIN_PERIOD 3
// Time after a button press that new button presses will be ignored (ms).
#define BTN_PRESS_DELAY 500

/**
 * Serial Definitions
 *
 * Defines baud rates and which Serial sould be used for each application.
 */

// Baud rate of the serial used on ros-serial.
#define BAUD_RATE_CONTROL 500000
// Baud rate of the serial used to communicate with servos.
#define BAUD_RATE_SERVOS 115200
// Baud rate of the debug serial.
#define BAUD_RATE_DEBUG 115200
// The Serial instance used with ros-serial.
#define SERIAL_CONTROL Serial

#ifdef USE_STM32_HW_SERIAL
// The Serial instance used to communicate with servos.
#define SERIAL_SERVOS Serial1
// The Serial instance used to debug.
#define SERIAL_DEBUG Serial2
#else
// The Serial instance used to communicate with servos.
#define SERIAL_SERVOS Serial2
// The Serial instance used to debug.
#define SERIAL_DEBUG Serial3
#endif

/**
 * Pin Definitions
 *
 * Defines used to map pins to specific functions.
 */

#define I2C_SCL PB8
#define I2C_SDA PB9
// Input pin of button0 from io board.
#define BUTTON0 PB12
// Input pin of button1 from io board.
#define BUTTON1 PB13
// Input pin of button2 from io board.
#define BUTTON2 PB14
// Input pin of button3 from io board.
#define BUTTON3 PB15
// Input pin of button4 from io board.
#define BUTTON4 PA8
// Output pin for LED0 on io board.
#define LED0 PB1
// Output pin for LED1 on io board.
#define LED1 PB0
// Output pin for LED2 on io board.
#define LED2 PA7
// Output pin for LED3 on io board.
#define LED3 PA6
// Output pin for LED4 on io board.
#define LED4 PA5
// Output pin of the led used to indicate the ros-serial connection.
#define LED_CONNECTION LED0
// Output pin of the led used to indicate the control mode (manual or auto).
#define LED_CONTROL_MODE LED1
// Output pin of the led used to indicate if the ServoManager is ready.
#define LED_READY LED2
// Output pin of the led used for debug purposes.
#define LED_DEBUG LED4

// Servos definitions
#define RIGHT_ANKLE_ROLL 0
#define RIGHT_ANKLE_PITCH 1
#define RIGHT_KNEE 2
#define RIGHT_HIP_PITCH 3
#define RIGHT_HIP_ROLL 4
#define RIGHT_HIP_YAW 5
#define LEFT_ANKLE_ROLL 6
#define LEFT_ANKLE_PITCH 7
#define LEFT_KNEE 8
#define LEFT_HIP_PITCH 9
#define LEFT_HIP_ROLL 10
#define LEFT_HIP_YAW 11
#define LEFT_ARM_PITCH 12
#define LEFT_ARM_YAW 13
#define LEFT_ARM_ROLL 14
#define RIGHT_ARM_PITCH 15
#define RIGHT_ARM_YAW 16
#define RIGHT_ARM_ROLL 17

#define RIGHT_ANKLE_ROLL_ID 5
#define RIGHT_ANKLE_PITCH_ID 10
#define RIGHT_KNEE_ID 16
#define RIGHT_HIP_PITCH_ID 4
#define RIGHT_HIP_ROLL_ID 15
#define RIGHT_HIP_YAW_ID 12
#define LEFT_ANKLE_ROLL_ID 1
#define LEFT_ANKLE_PITCH_ID 8
#define LEFT_KNEE_ID 14
#define LEFT_HIP_PITCH_ID 2
#define LEFT_HIP_ROLL_ID 9
#define LEFT_HIP_YAW_ID 6
#define LEFT_ARM_PITCH_ID 3
#define LEFT_ARM_YAW_ID 7
#define LEFT_ARM_ROLL_ID 11
#define RIGHT_ARM_PITCH_ID 13
#define RIGHT_ARM_YAW_ID 17
#define RIGHT_ARM_ROLL_ID 18

#define CHECK_ID RIGHT_HIP_YAW
#define POS_MIN -1650
#define POS_MAX 1650
#define XYZ_POS_MIN 0
#define XYZ_POS_MAX 1023
#define NUM_SERVOS 18
#define PLAYTIME 5
#define PLAYTIME_SMOOTH 100
#define DELAY_PLAYTIME (PLAYTIME * 10)
#define DELAY_PLAYTIME_SMOOTH (PLAYTIME_SMOOTH * 10)

// XYZ Protocol
#define XYZ_HEADER 0xFF
#define BROADCAST_ID 254
#define SET_POSITION_CONTROL 0
#define SET_TORQUE_OFF 2

#define SJOG_CMD 0x06
#define SJOG_HEADER_SIZE 8
#define SJOG_DATA_SIZE (4 * NUM_SERVOS)
#define SJOG_SIZE (SJOG_HEADER_SIZE + SJOG_DATA_SIZE)

#define IJOG_CMD 0x05
#define IJOG_HEADER_SIZE 7
#define IJOG_DATA_SIZE (5 * NUM_SERVOS)
#define IJOG_SIZE (IJOG_HEADER_SIZE + IJOG_DATA_SIZE)

#endif
