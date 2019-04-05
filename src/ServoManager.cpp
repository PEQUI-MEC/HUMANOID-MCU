#include <ServoManager.h>

ServoManager::ServoManager()
    : servos{{BodyServo(1, 0, 0, false), BodyServo(1, 0, 0, false),
              BodyServo(1, 0, 0, false), BodyServo(1, 0, 0, false),
              BodyServo(1, 0, 0, false), BodyServo(1, 0, 0, false),
              BodyServo(1, 0, 0, false), BodyServo(1, 0, 0, false),
              BodyServo(1, 0, 0, false), BodyServo(1, 0, 0, false),
              BodyServo(1, 0, 0, false), BodyServo(1, 0, 0, false),
              BodyServo(1, 0, 0, false), BodyServo(1, 0, 0, false),
              BodyServo(1, 0, 0, false), BodyServo(1, 0, 0, false),
              BodyServo(1, 0, 0, false), BodyServo(1, 0, 0, false)}} {}

ServoManager::~ServoManager() {}

BodyServo ServoManager::get_servo(uint8_t id) {
  for (uint8_t i = 0; i < servos.size(); i++) {
    if (servos[i].get_id() == id)
      return servos[i];
  }

  return 0;
}

void ServoManager::set_position(uint8_t id, int16_t position) {
  servos[id].set_position(position);
}

void ServoManager::assemble_pos_cmd(uint8_t* cmd) {
  uint8_t index;
  uint16_t abs_pos;

  // I-JOG: 0xFF 0xFF size id 0x05 checksum1 checksum2
  cmd[0] = 0xFF;
  cmd[1] = 0xFF;
  cmd[2] = POS_CMD_SIZE;
  cmd[3] = 254;
  cmd[4] = 0x05;

  for (uint8_t i = 0; i < servos.size(); i++) {
    index = POS_CMD_HEADER_SIZE + (5 * i);
    BodyServo& servo = servos[i];
    abs_pos = servo.get_abs_position();

    cmd[index] = abs_pos & 0xFF;
    cmd[index + 1] = abs_pos >> 8 & 0xFF;
    cmd[index + 2] = 0x00;
    cmd[index + 3] = servo.get_id();
    cmd[index + 4] = PLAYTIME;
  }

  uint8_t checksum = POS_CMD_DATA_SIZE;
  for (uint8_t i = 0; i < POS_CMD_DATA_SIZE; i++)
    checksum ^= cmd[i + POS_CMD_HEADER_SIZE];

  cmd[5] = checksum;
  cmd[6] = ~checksum;
}
