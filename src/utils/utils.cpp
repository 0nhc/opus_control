#include <utils/utils.h>
#include <stdint.h>
#include <algorithm>
#include <stdexcept>

// Limit the maximum and minimum values of data
float _limit(float input, float min, float max){
    return std::min(std::max(input, min), max);
};

uint16_t float_to_uint16(float x, float x_min, float x_max) {
    int bits = 16;
    
    // Validate the range
    if (x_min >= x_max) {
        throw std::invalid_argument("x_min must be less than x_max");
    }

    // Clamp the value to the specified range
    x = _limit(x, x_min, x_max);

    // Normalize and scale to the integer range [0, 2^bits - 1]
    float span = x_max - x_min;
    uint32_t max_int_value = (1U << bits) - 1;  // Use unsigned integer for bit-shifting
    return static_cast<uint16_t>((x - x_min) * max_int_value / span);
}
