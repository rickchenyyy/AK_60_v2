#ifndef MOTOR_H
#define MOTOR_H

#include <stdint.h>
#include "mainpp.h"
#include "controller.h"

#define MIN_POS_RESLOUTION 4E-4 // rad

struct MotorParams
{
    uint8_t id;
    float gohome_tolerance[2];
    float warning_pos[2];
    float fatal_pos[2];
    float torque_limit[2];
};

/**
 * @brief The Motor class represents a motor with various control and feedback mechanisms.
 *
 * @param param The parameters of the motor.
 * @param C1 The first controller object.
 * @param C2 The second controller object.
 */
class Motor
{
    friend class Controller;

public:
    Motor(const MotorParams param, const Controller &C1, const Controller &C2);
    uint8_t id;

    // Protection variables
    float gohome_tolerance[2]; // unit: rad, range of tolerance away from home (home as center)
    float warning_pos[2];      // unit: rad
    float fatal_pos[2];        // unit: rad
    float torque_limit[2];     // unit: Nm

    // Command and feedback
    float pos_command;    // pos after reducer, unit: rad
    float torque_command; // unit: Nm
    float pos_feedback;   // unit: rad
    float velocity_feedback;
    float torque_feedback;

    // Status
    float home;
    bool is_home;
    bool is_warning;
    bool is_fatal;

    // CAN message
    uint8_t Torque_cmd_tx[8];

    // Controller
    Controller controllers[2];

    // Methods
    void setTorqueCommand(float torque);
    void setPosCommand(float pos);
    /** @return 0 if normal, -1 if warning, -2 if fatal  */
    int checkStatus(); // return 0 if normal, -1 if warning, -2 if fatal
};

/**
 * @brief Check if all motors are at home position.
 * 
 * @return true if all motors are at home position, false otherwise.
 */
bool allAtHome();

#endif // MOTOR_H
