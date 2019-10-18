#include <ServoManager.h>

ServoManager manager;

ServoManager::ServoManager(ManagerState start_state)
    : servos{{BodyServo(RIGHT_ANKLE_ROLL, RIGHT_ANKLE_ROLL_ID, 0, 10, false),
              BodyServo(RIGHT_ANKLE_PITCH, RIGHT_ANKLE_PITCH_ID, 0, 0, true),
              BodyServo(RIGHT_KNEE, RIGHT_KNEE_ID, 0, -20, false),
              BodyServo(RIGHT_HIP_PITCH, RIGHT_HIP_PITCH_ID, 0, -5, false),
              BodyServo(RIGHT_HIP_ROLL, RIGHT_HIP_ROLL_ID, 0, -30, false),
              BodyServo(RIGHT_HIP_YAW, RIGHT_HIP_YAW_ID, 0, 0, true),
              BodyServo(LEFT_ANKLE_ROLL, LEFT_ANKLE_ROLL_ID, 0, -60, false),
              BodyServo(LEFT_ANKLE_PITCH, LEFT_ANKLE_PITCH_ID, 0, 10, true),
              BodyServo(LEFT_KNEE, LEFT_KNEE_ID, 0, 0, false),
              BodyServo(LEFT_HIP_PITCH, LEFT_HIP_PITCH_ID, 0, -40, false),
              BodyServo(LEFT_HIP_ROLL, LEFT_HIP_ROLL_ID, 0, -15, false),
              BodyServo(LEFT_HIP_YAW, LEFT_HIP_YAW_ID, 0, 0, true),
              BodyServo(LEFT_ARM_PITCH, LEFT_ARM_PITCH_ID, 0, 0, false),
              BodyServo(LEFT_ARM_YAW, LEFT_ARM_YAW_ID, 0, 0, false),
              BodyServo(LEFT_ARM_ROLL, LEFT_ARM_ROLL_ID, 0, 0, false),
              BodyServo(RIGHT_ARM_PITCH, RIGHT_ARM_PITCH_ID, 0, 0, false),
              BodyServo(RIGHT_ARM_YAW, RIGHT_ARM_YAW_ID, 0, 0, false),
              BodyServo(RIGHT_ARM_ROLL, RIGHT_ARM_ROLL_ID, 0, 0, false)}},
      serial(DMA1, DMA_CH7, SJOG_SIZE, DMA_IRQ_HANDLER_1),
      delay(DELAY_PLAYTIME_SMOOTH) {
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
  enable = true;
  smooth = false;

  switch (state) {
    case ManagerState::WaitServo:
      enable = false;
      break;

    case ManagerState::Initial:
    case ManagerState::SendSmoothIdle:
      smooth = true;
      break;

    case ManagerState::Ready:
      digitalWrite(LED_READY, HIGH);
      break;
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

void ServoManager::reset() {
  set_state(ManagerState::Initial);
  digitalWrite(LED_READY, LOW);

  for (uint8_t i = 0; i < servos.size(); i++)
    servos[i].set_position(0);
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
  if (serial.is_transfering() || !enable || !delay.has_finished())
    return false;

  assemble_pos_cmd(cmd_buffer);
  serial.set_data(cmd_buffer, SJOG_SIZE);
  if (serial.start() != DMA_TUBE_CFG_SUCCESS)
    return false;

  delay.start(smooth ? DELAY_PLAYTIME_SMOOTH : DELAY_PLAYTIME);

  // toggle_pin(LED_BUILTIN);
  return true;
}
