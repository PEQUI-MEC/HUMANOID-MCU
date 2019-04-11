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
#define IJOG_CMD 0x05
#define XYZ_POSITION_CONTROL 0
#define POS_CMD_HEADER_SIZE 7
#define POS_CMD_DATA_SIZE (5 * NUM_SERVOS)
#define POS_CMD_SIZE (POS_CMD_HEADER_SIZE + POS_CMD_DATA_SIZE)

#endif
