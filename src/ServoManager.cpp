#include <ServoManager.h>

ServoManager::ServoManager(ManagerState start_state)
    : servos{{BodyServo(1, 5, 0, -50, false), BodyServo(2, 10, 0, -90, true),
              BodyServo(3, 3, 0, -10, false), BodyServo(4, 4, 0, -160, false),
              BodyServo(5, 15, 0, -20, false), BodyServo(6, 12, 0, 0, true),
              BodyServo(7, 1, 0, -30, false), BodyServo(8, 8, 0, -90, true),
              BodyServo(9, 14, 0, -10, false), BodyServo(10, 2, 0, -180, false),
              BodyServo(11, 9, 0, 0, false), BodyServo(12, 6, 0, 0, true),
              BodyServo(13, 13, 0, 0, false), BodyServo(14, 7, 0, 0, false),
              BodyServo(15, 11, 0, 0, false), BodyServo(16, 16, 0, 0, false),
              BodyServo(17, 17, 0, 0, false), BodyServo(18, 18, 0, 0, false)}},
      serial(DMA1, DMA_CH7, SJOG_SIZE, DMA_IRQ_HANDLER_1) {
  set_state(start_state);
  wait_time = 0;
  wait_start = 0;
  torque = true;
  cmd_buffer = new uint8_t[SJOG_SIZE];
}

ServoManager::~ServoManager() {
  delete[] cmd_buffer;
}

void ServoManager::set_state(ManagerState state) {
  this->state = state;

  switch (state) {
    case ManagerState::Initial:
    case ManagerState::SendSmoothIdle:
      smooth = true;
      break;

    default:
      smooth = false;
  }
}

void ServoManager::state_logic() {
  switch (state) {
    case ManagerState::WaitServo:
      if (is_servo_connected())
        set_state(ManagerState::Initial);
      break;

    case ManagerState::IdleReceived:
      if (has_finished_waiting())
        set_state(ManagerState::SendSmoothIdle);
      break;

    case ManagerState::SendSmoothIdle:
      set_state(ManagerState::WaitSmoothIdle);
      break;

    case ManagerState::WaitSmoothIdle:
      if (has_finished_waiting())
        set_state(ManagerState::Ready);
      break;

    case ManagerState::Ready:
      // Send ready command
      set_state(ManagerState::Running);
      break;
  }
}

void ServoManager::updated_joint_pos() {
  if (state == ManagerState::Initial)
    set_state(ManagerState::IdleReceived);
}

void ServoManager::wait(time_t ms) {
  wait_time = ms;
  wait_start = millis();
}

bool ServoManager::has_finished_waiting() {
  if (wait_start == 0)
    return true;

  if (millis() - wait_start >= wait_time) {
    wait_time = 0;
    wait_start = 0;
    return true;
  }

  return false;
}

uint8_t ServoManager::get_servo_index(uint8_t cid) {
  for (uint8_t i = 0; i < servos.size(); i++) {
    if (servos[i].get_cid() == cid)
      return i;
  }

  return -1;
}

void ServoManager::set_position(uint8_t cid, int16_t position) {
  uint8_t i = get_servo_index(cid);
  if (i < 0)
    return;

  servos[i].set_position(position);
}

bool ServoManager::is_servo_connected(uint8_t rid) {
  if (rid == 0)
    rid = servos[get_servo_index(1)].get_rid();

  XYZrobotServo servo(SERIAL_SERVOS, rid);
  two_stage_blink(LED_BUILTIN, true);
  servo.readStatus();
  return !servo.getLastError();
}

void ServoManager::assemble_pos_cmd(uint8_t* buffer) {
  // S-JOG: 0xFF 0xFF size id cmd checksum1 checksum2 playtime
  buffer[0] = XYZ_HEADER;
  buffer[1] = XYZ_HEADER;
  buffer[2] = SJOG_SIZE;
  buffer[3] = BROADCAST_ID;
  buffer[4] = SJOG_CMD;
  buffer[7] = smooth ? PLAYTIME_SMOOTH : PLAYTIME;

  uint8_t index;
  uint16_t abs_pos;
  for (uint8_t i = 0; i < servos.size(); i++) {
    index = SJOG_HEADER_SIZE + (4 * i);
    abs_pos = servos[i].get_abs_position();

    buffer[index] = abs_pos & 0xFF;
    buffer[index + 1] = abs_pos >> 8 & 0xFF;
    buffer[index + 2] = torque ? SET_POSITION_CONTROL : SET_TORQUE_OFF;
    buffer[index + 3] = servos[i].get_rid();
  }

  uint8_t checksum = buffer[2] ^ buffer[3] ^ buffer[4] ^ buffer[7];
  for (uint8_t i = 0; i < SJOG_DATA_SIZE; i++)
    checksum ^= buffer[i + SJOG_HEADER_SIZE];

  buffer[5] = checksum & 0xFE;
  buffer[6] = ~checksum & 0xFE;
}

bool ServoManager::send_pos_cmd() {
  if (serial.is_transfering() || !has_finished_waiting() ||
      state == ManagerState::WaitServo)
    return false;

  assemble_pos_cmd(cmd_buffer);
  serial.set_data(cmd_buffer, SJOG_SIZE);
  if (serial.start() != DMA_TUBE_CFG_SUCCESS)
    return false;

  if (smooth)
    wait(PLAYTIME_SMOOTH * 10);

  toggle_pin(LED_BUILTIN);
  toggle_pin(LED4);
  return true;
}
