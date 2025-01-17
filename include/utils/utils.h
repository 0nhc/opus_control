#ifndef UTILS_H
#define UTILS_H

#include <stdint.h>

// Limit the maximum and minimum values of data
float _limit(float input, float min, float max);
uint16_t float_to_uint16(float x, float x_min, float x_max);

#endif