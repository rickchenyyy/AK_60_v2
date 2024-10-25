#include "config.h"

Controller m0_controller1("position_pd_w_compensation");
Controller m0_controller2("position_pi");

// Define parameters for motor 0
MotorParams motorparam0 = {
    0x0D,                          // Motor ID
    {0, deg2rad(20)},              // Go home tolerance
    {deg2rad(-60), deg2rad(60)},   // Warning pos range
    {deg2rad(-120), deg2rad(120)}, // Fatal pos range
    {-1, 1}                        // Torque limit range
};
Motor motor0(motorparam0, m0_controller1, m0_controller2);

Controller m1_controller1("position_pd_w_compensation");
Controller m1_controller2("position_pi");

MotorParams motorparam1 = {
    0x07,                          // Motor ID
    {0, deg2rad(20)},              // Go home tolerance
    {deg2rad(-60), deg2rad(60)},   // Warning pos range
    {deg2rad(-120), deg2rad(120)}, // Fatal pos range
    {-1, 1}                        // Torque limit range
};
Motor motor1(motorparam1, m1_controller1, m1_controller2);

// Create an array of motor objects
Motor motors[NUM_MOTORS] = {
    motor0,
    // motor1
};
