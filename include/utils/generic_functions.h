#ifndef GENERICFUNCTIONS_H_
#define GENERICFUNCTIONS_H_

#include <Arduino.h>
#include <config.h>

/**
 * Maps a value from one range to another range.
 *
 * A value of in_min would be mapped to out_min and a value of in_max would be
 * mapped to out_max. A value less then in_min is considered as in_min and a
 * value larger then in_max is considered as in_max.
 *
 * @param val Value that will be mapped from the in range to the out range.
 * @param in_min Minimum value of the in range.
 * @param in_max Maximum value of the in range.
 * @param out_min Minimum value of the out range.
 * @param out_max Maximum value of the out range.
 */
float range_map(float val,
                float in_min,
                float in_max,
                float out_min,
                float out_max);

/**
 * Generates a position based on a sine-like movement and the current time.
 *
 * @param min Inferior position limit of the moviment.
 * @param max Superior position limit of the moviment.
 * @param f Frequency of the sine-like movement.
 * @return The position generated.
 */
int16_t sine_position(int16_t min = POS_MIN,
                      int16_t max = POS_MAX,
                      float f = 0.5);

/**
 * Toggles the output of a pin.
 *
 * If the output is HIGH, sets it to LOW. If the output is LOW, sets it to HIGH.
 *
 * @param pin Number of the pin to toggle.
 */
void toggle_pin(uint8_t pin);

/**
 * Blinks a pin with two stages.
 *
 * In the first stage the pin is activated for a long period and in the second
 * stage the pin is activated for a short period.
 *
 * @param pin Number of the pin to blink.
 * @param reverse Should be true if the active state of the pin is LOW.
 */
void two_stage_blink(uint8_t pin, bool reverse = false);

#endif
