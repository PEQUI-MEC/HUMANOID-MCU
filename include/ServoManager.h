#ifndef SERVOMANAGER_H_
#define SERVOMANAGER_H_

#undef min  // Workaround para corrigir erro de compilação
#undef max  // Workaround para corrigir erro de compilação
#include <array>

#include <Arduino.h>
#include <BodyServo.h>
#include <DMASerial.h>
#include <XYZrobotServo.h>
#include <config.h>
#include <peripherals.h>
#include <utils/DelayTimer.h>
#include <utils/generic_functions.h>

enum class ManagerState {
  WaitServo,
  Initial,
  IdleReceived,
  SendSmoothIdle,
  WaitSmoothIdle,
  Ready,
  Running
};

class ServoManager {
 private:
  ManagerState state;
  std::array<BodyServo, NUM_SERVOS> servos;

 public:
  DMASerial serial;
  DelayTimer delay;
  uint8_t* cmd_buffer;
  bool enable;
  bool torque;
  bool smooth;

  ServoManager(ManagerState start_state = ManagerState::WaitServo);
  ~ServoManager();

  ManagerState get_state();
  void set_state(ManagerState state);
  void state_logic();
  void reset();

  uint8_t get_servo_index(uint8_t cid);
  void set_position(uint8_t cid, int16_t position);
  bool is_servo_connected(uint8_t id = CHECK_ID, bool translate_id = true);

  void assemble_pos_cmd(uint8_t* cmd_data);
  bool send_pos_cmd();
};

extern ServoManager manager;

#endif
