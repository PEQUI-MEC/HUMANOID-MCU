#ifndef BODYSERVO_H_
#define BODYSERVO_H_

#include <Arduino.h>
#include <config.h>
#include <utils/generic_functions.h>

/**
 * A class that represents a servo in the body.
 */
class BodyServo {
 private:
  uint8_t cid;       // The ID that comes from the gait control.
  uint8_t rid;       // Real ID of the servo.
  int16_t position;  // Position goal in degrees.
  int16_t offset;    // Position offset in degrees.
  bool reverse;      // Flags if the movement should be reversed.

 public:
  /**
   * @param cid Gait Control ID.
   * @param rid Real servo ID.
   * @param position Initial position of the servo in degrees.
   * @param offset Position offset in degrees.
   * @param reverse If the movement should be reversed.
   */
  explicit BodyServo(uint8_t cid,
                     uint8_t rid,
                     int16_t position = 0,
                     int16_t offset = 0,
                     bool reverse = false);
  ~BodyServo();

  /**
   * @return Gait control ID of this servo.
   */
  uint8_t get_cid();

  /**
   * @return Real servo ID.
   */
  uint8_t get_rid();

  /**
   * @return Position goal in degrees.
   */
  int16_t get_position(void);

  /**
   * Calculates and returns the absolute position in servo units (between 0 and
   * 1024).
   *
   * @return Absolute position in servo units.
   */
  uint16_t get_abs_position(void);

  /**
   * Sets the goal position in degrees of this servo.
   *
   * @param pos Goal position in degrees.
   */
  void set_position(int16_t pos);
};

#endif
