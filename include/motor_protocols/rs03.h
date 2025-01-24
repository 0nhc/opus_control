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

#define MASTER_CAN_ID 0x00

#define MOTION_CONTROL_MODE 0x01  // Motion Control Mode
#define ENABLE_MODE 0x03  // Enable Mode

class RS03
{
    public:
        RS03();
        can_msgs::msg::Frame encodeEnableCommand(uint8_t motor_id);
        can_msgs::msg::Frame encodeTorqueCommand(uint8_t motor_id, float torque);

        float decodePositionFeedback(can_msgs::msg::Frame frame);

    private:
        uint16_t _master_can_id;
        can_msgs::msg::Frame _enable_command;
        can_msgs::msg::Frame _torque_command;
};

#endif