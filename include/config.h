#ifndef CONFIG_H_
#define CONFIG_H_

#define INITIAL_DELAY 2000
#define SPIN_PERIOD 10  // ms

// Servos definitions
#define NUM_SERVOS 18
#define PLAYTIME 1
#define POS_CMD_HEADER_SIZE 7
#define POS_CMD_DATA_SIZE (5 * NUM_SERVOS)
#define POS_CMD_SIZE (POS_CMD_HEADER_SIZE + POS_CMD_DATA_SIZE)

#endif
