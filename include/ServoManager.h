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
  Initial = 0,
  IdleReceived = 1,
  SendSmoothIdle = 2,
  WaitSmoothIdle = 3,
  Ready = 4,
  Running = 5
};

class ServoManager {
 private:
  ManagerState state;
  time_t wait_time;
  time_t wait_start;
  bool smooth;

  std::array<BodyServo, NUM_SERVOS> servos;

 public:
  DMASerial serial;
  uint8_t* cmd_buffer;
  bool torque;

  ServoManager();
  ~ServoManager();

  void set_state(ManagerState state);
  void state_logic();
  void updated_joint_pos();

  void wait(time_t ms);
  void wait_servo_connection(uint8_t id = 0);
  bool has_finished_waiting();

  uint8_t get_servo_index(uint8_t cid);
  void set_position(uint8_t cid, int16_t position);

  void assemble_pos_cmd(uint8_t* cmd_data);
  bool send_pos_cmd();
};

#endif
