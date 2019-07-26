#include <ServoManager.h>

ServoManager::ServoManager()
    : servos{{BodyServo(1, 5, 0, 0, true), BodyServo(2, 4, 0, 0, false),
              BodyServo(3, 3, 0, 0, true), BodyServo(4, 10, 0, 0, true),
              BodyServo(5, 15, 0, 0, true), BodyServo(6, 6, 0, 0, true),
              BodyServo(7, 1, 0, 0, true), BodyServo(8, 8, 0, 0, true),
              BodyServo(9, 7, 0, 0, false), BodyServo(10, 2, 0, 0, false),
              BodyServo(11, 18, 0, 0, true), BodyServo(12, 12, 0, 0, true),
              BodyServo(13, 13, 0, 0, false), BodyServo(14, 14, 0, 0, false),
              BodyServo(15, 9, 0, 0, false), BodyServo(16, 16, 0, 0, false),
              BodyServo(17, 17, 0, 0, false), BodyServo(18, 11, 0, 0, false)}} {
  state = STATE_INITIAL;
  delay = 0;
  delay_start = 0;
}

ServoManager::~ServoManager() {}

void ServoManager::set_state(uint8_t new_state) {
  if (state == STATE_INITIAL || new_state == STATE_INITIAL)
    setup_delay(PLAYTIME_SMOOTH * 10);

  state = new_state;
}

void ServoManager::setup_delay(time_t delay) {
  this->delay = delay;
}

void ServoManager::start_delay() {
  if (delay > 0)
    delay_start = millis();
}

bool ServoManager::reset_delay() {
  if (delay_start == 0) {
    return true;
  } else if (millis() - delay_start >= delay) {
    delay = 0;
    delay_start = 0;
    return true;
  } else {
    return false;
  }
}

uint8_t ServoManager::get_index(uint8_t cid) {
  for (uint8_t i = 0; i < servos.size(); i++) {
    if (servos[i].get_cid() == cid)
      return i;
  }

  return -1;
}

void ServoManager::set_position(uint8_t cid, int16_t position) {
  uint8_t i = get_index(cid);
  if (i < 0)
    return;

  servos[i].set_position(position);
}

void ServoManager::assemble_pos_cmd(uint8_t* cmd) {
  // S-JOG: 0xFF 0xFF size id cmd checksum1 checksum2 playtime
  cmd[0] = XYZ_HEADER;
  cmd[1] = XYZ_HEADER;
  cmd[2] = SJOG_SIZE;
  cmd[3] = BROADCAST_ID;
  cmd[4] = SJOG_CMD;
  cmd[7] = max(PLAYTIME, delay / 10);

  uint8_t index;
  uint16_t abs_pos;
  for (uint8_t i = 0; i < servos.size(); i++) {
    index = SJOG_HEADER_SIZE + (4 * i);
    BodyServo& servo = servos[i];
    abs_pos = servo.get_abs_position();

    cmd[index] = abs_pos & 0xFF;
    cmd[index + 1] = abs_pos >> 8 & 0xFF;
    cmd[index + 2] = XYZ_POSITION_CONTROL;
    cmd[index + 3] = servo.get_rid();
  }

  uint8_t checksum = cmd[2] ^ cmd[3] ^ cmd[4] ^ cmd[7];
  for (uint8_t i = 0; i < SJOG_DATA_SIZE; i++)
    checksum ^= cmd[i + SJOG_HEADER_SIZE];

  cmd[5] = checksum & 0xFE;
  cmd[6] = ~checksum & 0xFE;

  start_delay();
}
