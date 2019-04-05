#ifndef SERVOUTILS_H_
#define SERVOUTILS_H_

// int range_map(int val, int in_min, int in_max, int out_min, int out_max);
float range_map(float val,
                float in_min,
                float in_max,
                float out_min,
                float out_max);

#endif
