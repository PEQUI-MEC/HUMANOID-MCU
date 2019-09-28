#include <ServoManager.h>

ServoManager manager;

ServoManager::ServoManager(ManagerState start_state)
    : servos{{BodyServo(RIGHT_ANKLE_ROLL, 5, 0, -30, false),
              BodyServo(RIGHT_ANKLE_PITCH, 10, 0, 0, true),
              BodyServo(RIGHT_KNEE, 3, 0, -20, false),
              BodyServo(RIGHT_HIP_PITCH, 4, 0, -5, false),
              BodyServo(RIGHT_HIP_ROLL, 15, 0, -30, false),
              BodyServo(RIGHT_HIP_YAW, 12, 0, 0, true),
              BodyServo(LEFT_ANKLE_ROLL, 1, 0, -30, false),
              BodyServo(LEFT_ANKLE_PITCH, 8, 0, -30, true),
              BodyServo(LEFT_KNEE, 14, 0, -10, false),
              BodyServo(LEFT_HIP_PITCH, 2, 0, -20, false),
              BodyServo(LEFT_HIP_ROLL, 9, 0, -15, false),
              BodyServo(LEFT_HIP_YAW, 6, 0, 0, true),
              BodyServo(LEFT_ARM_PITCH, 13, 0, 0, false),
              BodyServo(LEFT_ARM_YAW, 7, 0, 0, false),
              BodyServo(LEFT_ARM_ROLL, 11, 0, 0, false),
              BodyServo(RIGHT_ARM_PITCH, 16, 0, 0, false),
              BodyServo(RIGHT_ARM_YAW, 17, 0, 0, false),
              BodyServo(RIGHT_ARM_ROLL, 18, 0, 0, false)}},
      serial(DMA1, DMA_CH7, SJOG_SIZE, DMA_IRQ_HANDLER_1),
      delay(PLAYTIME_SMOOTH * 10) {
  set_state(start_state);
  torque = true;
  cmd_buffer = new uint8_t[SJOG_SIZE];
}

ServoManager::~ServoManager() {
  delete[] cmd_buffer;
}

ManagerState ServoManager::get_state() {
  return state;
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
      if (delay.has_finished())
        set_state(ManagerState::SendSmoothIdle);
      break;

    case ManagerState::SendSmoothIdle:
      set_state(ManagerState::WaitSmoothIdle);
      break;

    case ManagerState::WaitSmoothIdle:
      if (delay.has_finished())
        set_state(ManagerState::Ready);
      break;

    case ManagerState::Ready:
      control.publish_command("ready");
      set_state(ManagerState::Running);
      break;
  }
}

void ServoManager::updated_joint_pos() {
  if (state == ManagerState::Initial)
    set_state(ManagerState::IdleReceived);
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

bool ServoManager::is_servo_connected(uint8_t id, bool translate_id) {
  if (translate_id)
    id = servos[get_servo_index(id)].get_rid();

  XYZrobotServo servo(SERIAL_SERVOS, id);
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
  if (serial.is_transfering() || !delay.has_finished() ||
      state == ManagerState::WaitServo)
    return false;

  assemble_pos_cmd(cmd_buffer);
  serial.set_data(cmd_buffer, SJOG_SIZE);
  if (serial.start() != DMA_TUBE_CFG_SUCCESS)
    return false;

  if (smooth)
    delay.start();

  // toggle_pin(LED_BUILTIN);
  return true;
}
