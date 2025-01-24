#include <motor_protocols/rs03.h>
#include <utils/utils.h>
#include <iostream>
#include <cmath>


RS03::RS03(){
    this->_master_can_id = MASTER_CAN_ID;
};

can_msgs::msg::Frame RS03::encodeEnableCommand(uint8_t motor_id){
    this->_enable_command.is_extended = true;
    this->_enable_command.is_error = false;
    this->_enable_command.is_rtr = false;
    this->_enable_command.dlc = 0x08; // 8 bits
    this->_enable_command.id = (ENABLE_MODE << 24) | // Communication type: Motion control (bit 28~24)
                               (this->_master_can_id << 8) | // Master CAN ID in bit 23~8
                               motor_id; // Motor ID in bit 7~0
    this->_enable_command.data[0] = 0x00;
    this->_enable_command.data[1] = 0x00;
    this->_enable_command.data[2] = 0x00;
    this->_enable_command.data[3] = 0x00;
    this->_enable_command.data[4] = 0x00;
    this->_enable_command.data[5] = 0x00;
    this->_enable_command.data[6] = 0x00;
    this->_enable_command.data[7] = 0x00;

    return this->_enable_command;
}

can_msgs::msg::Frame RS03::encodeTorqueCommand(uint8_t motor_id, float torque){
    // Limit the torque value to -1.0 to 1.0
    torque = _limit(torque, -0.6, 0.6);

    // Clamp values to their respective ranges
    uint16_t encoded_torque = float_to_uint16(torque, T_MIN, T_MAX);
    uint16_t encoded_angle = float_to_uint16(0.0, P_MIN, P_MAX);
    uint16_t encoded_speed = float_to_uint16(0.0, V_MIN, V_MAX);
    uint16_t encoded_Kp = float_to_uint16(0.0, KP_MIN, KP_MAX);
    uint16_t encoded_Kd = float_to_uint16(0.0, KD_MIN, KD_MAX);

    this->_torque_command.is_extended = true;
    this->_torque_command.is_error = false;
    this->_torque_command.is_rtr = false;
    this->_torque_command.dlc = 0x08; // 8 bits

    this->_torque_command.id = (MOTION_CONTROL_MODE << 24) | // Communication type: Motion control (bit 28~24)
                         (encoded_torque << 8) | // Torque in bit 23~8
                          motor_id; // Motor ID in bit 7~0

    // Fill the data payload (big-endian order)
    this->_torque_command.data[0] = (encoded_angle >> 8) & 0xFF; // MSB of angle
    this->_torque_command.data[1] = encoded_angle & 0xFF;        // LSB of angle
    this->_torque_command.data[2] = (encoded_speed >> 8) & 0xFF; // MSB of speed
    this->_torque_command.data[3] = encoded_speed & 0xFF;        // LSB of speed
    this->_torque_command.data[4] = (encoded_Kp >> 8) & 0xFF;    // MSB of Kp
    this->_torque_command.data[5] = encoded_Kp & 0xFF;           // LSB of Kp
    this->_torque_command.data[6] = (encoded_Kd >> 8) & 0xFF;    // MSB of Kd
    this->_torque_command.data[7] = encoded_Kd & 0xFF;           // LSB of Kd

    return this->_torque_command;
}

float RS03::decodePositionFeedback(can_msgs::msg::Frame frame){
    // Extract the angle from the CAN frame
    uint16_t angle = (frame.data[0] << 8) | frame.data[1];
    return (angle * (P_MAX - P_MIN) / 65535.0) + P_MIN;
}