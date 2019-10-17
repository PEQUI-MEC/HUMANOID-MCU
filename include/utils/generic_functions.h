#ifndef GENERICFUNCTIONS_H_
#define GENERICFUNCTIONS_H_

#include <Arduino.h>

// int range_map(int val, int in_min, int in_max, int out_min, int out_max);
float range_map(float val,
                float in_min,
                float in_max,
                float out_min,
                float out_max);

void toggle_pin(uint8_t pin);
void two_stage_blink(uint8_t pin, bool reverse = false);

#endif
