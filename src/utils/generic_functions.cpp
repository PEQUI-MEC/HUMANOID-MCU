#include <utils/generic_functions.h>

// int range_map(int val, int in_min, int in_max, int out_min, int out_max) {
//   if (val < in_min)
//     return out_min;
//   if (val > in_max)
//     return out_max;
//   return (val - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
// }

float range_map(float val,
                float in_min,
                float in_max,
                float out_min,
                float out_max) {
  if (val < in_min)
    return out_min;
  if (val > in_max)
    return out_max;
  return (val - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void toggle_pin(uint8_t pin) {
  digitalWrite(pin, !digitalRead(pin));
}
