#include "motor.h"
#include "can.h"
#include <cstring>

Motor::Motor(const MotorParams param, const Controller &C1, const Controller &C2)
    : id(param.id), controllers{C1, C2}
{
    // Initialize using parameters from MotorParams
    gohome_tolerance[0] = param.gohome_tolerance[0];
    gohome_tolerance[1] = param.gohome_tolerance[1];
    warning_pos[0] = param.warning_pos[0];
    warning_pos[1] = param.warning_pos[1];
    fatal_pos[0] = param.fatal_pos[0];
    fatal_pos[1] = param.fatal_pos[1];
    torque_limit[0] = param.torque_limit[0];
    torque_limit[1] = param.torque_limit[1];

    // Initialize command and feedback variables to zero
    pos_command = 0.0f;
    torque_command = 0.0f;
    pos_feedback = 0.0f;
    velocity_feedback = 0.0f;
    torque_feedback = 0.0f;

    // Initialize status variables
    home = 0.0f;
    is_home = false;
    is_warning = false;
    is_fatal = false;

    // Initialize torque command , see manual p48, 49
    uint8_t torque_command[8] = {0x7F, 0xFF, 0x7F, 0xF0, 0x00, 0x00, 0x00, 0x00}; // -0.3NM
    memcpy(Torque_cmd_tx, torque_command, sizeof(torque_command));
}

void Motor::setTorqueCommand(float torque)
{
    // Set the torque command
    torque_command = torque;
    // check torque limit
    if (torque_command < torque_limit[0])
    {
        torque_command = torque_limit[0];
    }
    else if (torque_command > torque_limit[1])
    {
        torque_command = torque_limit[1];
    }
    // Convert torque command to CAN message
    uint32_t torque_int = FLOAT_TO_INT_TORQUE(torque_command);
    Torque_cmd_tx[6] = (0x00 << 4) | (torque_int >> 8);
    Torque_cmd_tx[7] = (torque_int & 0xFF);
    return;
}

void Motor::setPosCommand(float pos)
{
    // check pos vadility
    if (pos < warning_pos[0] || pos > warning_pos[1])
    {
        printf("Motor %d setting invalid pos command: %.3f\n", id, pos);
    }
    else
    {
        pos_command = pos;
    }
    return;
}

int Motor::checkStatus()
{
    if (is_fatal)
    {
        // printf("Motor %d is in fatal status\n", id);
        return -2;
    }
    else if (is_warning)
    {
        // printf("Motor %d is in warning status\n", id);
        return -1;
    }
    else
    {
        // printf("Motor %d is in normal status\n", id);
        return 0;
    }
}

bool allAtHome()
{
    for (int i = 0; i < NUM_MOTORS; i++)
    {
        if (!motors[i].is_home)
        {
            return false;
        }
    }
    return true;
}
