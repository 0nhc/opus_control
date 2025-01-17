#ifndef RS03_H
#define RS03_H
// Motor protocols for RS03 motor. Fabricated by RobStride Dynamics.

#include "can_msgs/msg/frame.hpp"
#include <vector>

#define P_MIN -12.567f
#define P_MAX 12.567f
#define V_MIN -20.0f 
#define V_MAX 20.0f
#define KP_MIN 0.0f
#define KP_MAX 100.0f
#define KD_MIN 0.0f
#define KD_MAX 5.0f
#define T_MIN -60.0f
#define T_MAX 60.0f

class RS03
{
    public:
        RS03();
        can_msgs::msg::Frame encodeTorqueCommand(uint8_t motor_id, float torque);

    private:
        can_msgs::msg::Frame _torque_command;
};

#endif