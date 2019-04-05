#ifndef MOCKCONTROLLER_H_
#define MOCKCONTROLLER_H_

#include <Arduino.h>
#include <ServoManager.h>
#include <config.h>

class MockController {
 private:
  MockController();
  ~MockController();

 public:
  static void generate_sine_positions(ServoManager manager,
                                      int16_t min = -180,
                                      int16_t max = 150,
                                      float f = 0.5);
};

#endif
