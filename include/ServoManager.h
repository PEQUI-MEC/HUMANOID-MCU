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

enum class ManagerState {
  WaitServo = 0,
  Initial = 1,
  IdleReceived = 2,
  SendSmoothIdle = 3,
  WaitSmoothIdle = 4,
  Ready = 5,
  Running = 6
};

class ServoManager {
 private:
  ManagerState state;
  time_t wait_time;
  time_t wait_start;

  std::array<BodyServo, NUM_SERVOS> servos;

 public:
  DMASerial serial;
  uint8_t* cmd_buffer;
  bool torque;
  bool smooth;

  ServoManager(ManagerState start_state = ManagerState::WaitServo);
  ~ServoManager();

  ManagerState get_state();
  void set_state(ManagerState state);
  void state_logic();
  void updated_joint_pos();

  void wait(time_t ms);
  bool has_finished_waiting();

  uint8_t get_servo_index(uint8_t cid);
  void set_position(uint8_t cid, int16_t position);
  bool is_servo_connected(uint8_t id = CHECK_ID, bool translate_id = true);

  void assemble_pos_cmd(uint8_t* cmd_data);
  bool send_pos_cmd();
};

#endif
