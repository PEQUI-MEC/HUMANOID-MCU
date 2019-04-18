#ifndef CONFIG_H_
#define CONFIG_H_

#define INITIAL_DELAY 2000
#define SPIN_PERIOD 10  // ms

// Servos definitions
#define POS_MIN -1800
#define POS_MAX 1500
#define XYZ_POS_MIN 0
#define XYZ_POS_MAX 1023
#define NUM_SERVOS 18
#define PLAYTIME 1

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
