#ifndef DELAYTIMER_H_
#define DELAYTIMER_H_

#include <Arduino.h>

class DelayTimer {
 private:
  time_t delay;
  time_t start_time;
  bool is_running = false;

 public:
  DelayTimer(time_t delay = 0);
  ~DelayTimer();

  void set_delay(time_t delay);
  bool start();
  bool start(time_t delay);
  bool has_finished();
  void reset();
};

#endif
