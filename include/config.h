#ifndef CONFIG_H_
#define CONFIG_H_

#include <Arduino.h>

#define INITIAL_DELAY 3000
#define SPIN_PERIOD 5  // ms
#define BTN_PRESS_DELAY 500

// Serial definitions
#define BAUD_RATE_CONTROL 500000
#define BAUD_RATE_SERVOS 115200
#define BAUD_RATE_DEBUG 115200
#define SERIAL_CONTROL Serial

#ifdef USE_STM32_HW_SERIAL
#define SERIAL_SERVOS Serial1
#define SERIAL_DEBUG Serial2
#else
#define SERIAL_SERVOS Serial2
#define SERIAL_DEBUG Serial3
#endif

// Pin definitions
#define VBAT_ADC PA0  // 2.2V
#define I2C_SCL PB8
#define I2C_SDA PB9
#define BUTTON0 PB12
#define BUTTON1 PB13
#define BUTTON2 PB14
#define BUTTON3 PB15
#define BUTTON4 PA8
#define LED0 PB1
#define LED1 PB0
#define LED2 PA7
#define LED3 PA6
#define LED4 PA5

// Control States
#define STATE_INITIAL 0
#define STATE_IDLE 1
#define STATE_MARCH 2
#define STATE_WALK 3
#define STATE_TURN 4
#define STATE_FALLEN 5
#define STATE_UP 6
#define STATE_PENALIZED 7
#define STATE_TURN90 8

// Servos definitions
#define POS_MIN -1650
#define POS_MAX 1650
#define XYZ_POS_MIN 0
#define XYZ_POS_MAX 1023
#define NUM_SERVOS 18
#define PLAYTIME 1
#define PLAYTIME_SMOOTH 150

// XYZ Communication
#define XYZ_HEADER 0xFF
#define BROADCAST_ID 254
#define XYZ_POSITION_CONTROL 0

#define SJOG_CMD 0x06
#define SJOG_HEADER_SIZE 8
#define SJOG_DATA_SIZE (4 * NUM_SERVOS)
#define SJOG_SIZE (SJOG_HEADER_SIZE + SJOG_DATA_SIZE)

#define IJOG_CMD 0x05
#define IJOG_HEADER_SIZE 7
#define IJOG_DATA_SIZE (5 * NUM_SERVOS)
#define IJOG_SIZE (IJOG_HEADER_SIZE + IJOG_DATA_SIZE)

#endif
