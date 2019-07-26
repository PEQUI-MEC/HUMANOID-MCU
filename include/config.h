#ifndef CONFIG_H_
#define CONFIG_H_

#define INITIAL_DELAY 5000
#define SPIN_PERIOD 5  // ms
#define USB_BAUD_RATE 115200

// Servos definitions
#define POS_MIN -1650
#define POS_MAX 1650
#define XYZ_POS_MIN 0
#define XYZ_POS_MAX 1023
#define NUM_SERVOS 18
#define PLAYTIME 1
#define PLAYTIME_SMOOTH 150

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
