#include <ServoManager.h>

ServoManager::ServoManager()
    : servos{{BodyServo(1, 0, 0, false), BodyServo(2, 0, 0, false),
              BodyServo(3, 0, 0, false), BodyServo(4, 0, 0, false),
              BodyServo(5, 0, 0, false), BodyServo(6, 0, 0, false),
              BodyServo(7, 0, 0, false), BodyServo(8, 0, 0, false),
              BodyServo(9, 0, 0, false), BodyServo(10, 0, 0, false),
              BodyServo(11, 0, 0, false), BodyServo(12, 0, 0, false),
              BodyServo(13, 0, 0, false), BodyServo(14, 0, 0, false),
              BodyServo(15, 0, 0, false), BodyServo(16, 0, 0, false),
              BodyServo(17, 0, 0, false), BodyServo(18, 0, 0, false)}} {}

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
  cmd[0] = XYZ_HEADER;
  cmd[1] = XYZ_HEADER;
  cmd[2] = POS_CMD_SIZE;
  cmd[3] = BROADCAST_ID;
  cmd[4] = IJOG_CMD;

  for (uint8_t i = 0; i < servos.size(); i++) {
    index = POS_CMD_HEADER_SIZE + (5 * i);
    BodyServo& servo = servos[i];
    abs_pos = servo.get_abs_position();

    cmd[index] = abs_pos & 0xFF;
    cmd[index + 1] = abs_pos >> 8 & 0xFF;
    cmd[index + 2] = XYZ_POSITION_CONTROL;
    cmd[index + 3] = servo.get_id();
    cmd[index + 4] = PLAYTIME;
  }

  uint8_t checksum = cmd[2] ^ cmd[3] ^ cmd[4];
  for (uint8_t i = 0; i < POS_CMD_DATA_SIZE; i++)
    checksum ^= cmd[i + POS_CMD_HEADER_SIZE];

  cmd[5] = checksum & 0xFE;
  cmd[6] = ~checksum & 0xFE;
}
