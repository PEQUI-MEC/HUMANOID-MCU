#include <utils/DelayTimer.h>

DelayTimer::DelayTimer(time_t delay) : delay(delay) {}
DelayTimer::~DelayTimer() {}

void DelayTimer::set_delay(time_t delay) {
  this->delay = delay;
}

bool DelayTimer::start() {
  if (is_running)
    return false;

  start_time = millis();
  is_running = true;
  return true;
}

bool DelayTimer::start(time_t delay) {
  if (is_running)
    return false;

  set_delay(delay);
  return start();
}

bool DelayTimer::has_finished() {
  if (!is_running)
    return true;

  if (millis() - start_time < delay)
    return false;

  reset();
  return true;
}

void DelayTimer::reset() {
  is_running = false;
  start_time = 0;
}
