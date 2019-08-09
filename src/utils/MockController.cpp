#include <utils/MockController.h>

void MockController::generate_sine_positions(ServoManager& manager,
                                             int16_t min,
                                             int16_t max,
                                             float f) {
  uint16_t amplitude = (max - min) / 2;
  uint16_t reference = min + amplitude;
  float time = (float)millis() / 1000.0;

  int16_t pos = ((amplitude * cos(2 * M_PI * f * time)) + reference);
  manager.set_position(3, pos);
  manager.set_position(9, pos);
  // manager.set_position(2, pos);
  // manager.set_position(7, pos);
  // manager.set_position(4, pos);
  // manager.set_position(10, pos);
  // for (uint8_t i = 1; i <= NUM_SERVOS; i++)
  //   manager.set_position(i, pos);
}
