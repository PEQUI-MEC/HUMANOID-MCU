#ifndef DELAYTIMER_H_
#define DELAYTIMER_H_

#include <Arduino.h>

/**
 * A class representing a delay that does not block execution
 */
class DelayTimer {
 private:
  time_t delay;             // Amount of delay
  time_t start_time;        // When the delay started
  bool is_running = false;  // Flag if the delay has started and hasn't finished

 public:
  /**
   * @param delay Amount of delay.
   */
  DelayTimer(time_t delay = 0);
  ~DelayTimer();

  /**
   * Sets the amount of delay.
   *
   * @param delay Amount of delay
   */
  void set_delay(time_t delay);

  /**
   * Starts the delay if it is not running. Reads the millisseconds of the start
   * and stores in the start_time member.
   */
  bool start();

  /**
   * Starts the delay, if it is not running, with a different amount. Reads the
   * millisseconds of the start and stores in the start_time member.
   *
   * @param delay New amount of delay
   */
  bool start(time_t delay);

  /**
   * Checks if the delay has finished (current_time >= start_time + delay). If
   * it has finished, resets the delay.
   *
   * @return true if the delay has finished or if the delay is not running.
   * @see #reset()
   */
  bool has_finished();

  /**
   * Resets the start_time and is_running members to indicate the delay is no
   * longer active.
   */
  void reset();
};

#endif
