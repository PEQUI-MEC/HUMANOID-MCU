#ifndef SERVOMANAGER_H_
#define SERVOMANAGER_H_

#undef min  // Workaround para corrigir erro de compilação
#undef max  // Workaround para corrigir erro de compilação
#include <array>

#include <Arduino.h>
#include <BodyServo.h>
#include <config.h>

class ServoManager {
 private:
  std::array<BodyServo, NUM_SERVOS> servos;

 public:
  ServoManager();
  ~ServoManager();

  uint8_t get_index(uint8_t cid);
  BodyServo& get_servo(uint8_t cid);
  void set_position(uint8_t cid, int16_t position);

  void assemble_pos_cmd(uint8_t* cmd_data);
};

#endif
