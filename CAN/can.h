#ifndef CAN_H
#define CAN_H

#include "mainpp.h"
#include "config.h"
#include "utility.h"

// See manual p49-51
// Precompute scaling factors using #define for better performance
#define POS_MIN (-12.5f)
#define POS_MAX (12.5f)
#define POS_SCALE_FACTOR ((POS_MAX - POS_MIN) / 65535.0f)

#define VEL_MIN (-45.0f)
#define VEL_MAX (45.0f)
#define VEL_SCALE_FACTOR ((VEL_MAX - VEL_MIN) / 4095.0f)

#define TORQUE_MIN (-15.0f)
#define TORQUE_MAX (15.0f)
#define TORQUE_SCALE_FACTOR ((TORQUE_MAX - TORQUE_MIN) / 4095.0f)

// Define for integer to float conversions
#define INT_TO_FLOAT_POS(p_int) ((p_int * POS_SCALE_FACTOR) + POS_MIN)
#define INT_TO_FLOAT_VEL(v_int) ((v_int * VEL_SCALE_FACTOR) + VEL_MIN)
#define INT_TO_FLOAT_TORQUE(t_int) ((t_int * TORQUE_SCALE_FACTOR) + TORQUE_MIN)

// Define for float to integer conversions
#define FLOAT_TO_INT_POS(p_float) (uint32_t)(((p_float - POS_MIN) / (POS_MAX - POS_MIN)) * 65535.0f)
#define FLOAT_TO_INT_VEL(v_float) (uint32_t)(((v_float - VEL_MIN) / (VEL_MAX - VEL_MIN)) * 4095.0f)
#define FLOAT_TO_INT_TORQUE(t_float) (uint32_t)(((t_float - TORQUE_MIN) / (TORQUE_MAX - TORQUE_MIN)) * 4095.0f)

// See manual p48
// Define CANMessage structure
typedef struct
{
    int id;
    float pos;
    float vel;
    float torque;
    // int temp;
    int error_code;
} CANMessage;

void canInit();

/**
 * @brief Starts the motor.
 *
 * @return int Returns 0 if the motor starts successfully, -1 otherwise.
 */
int startMotor();

/*
 * @brief Stops the motor.
 */
void stopMotor();

/**
 * @brief Receives a CAN message.
 *
 * @param msg Pointer to a CANMessage structure where the received message will be stored.
 * @param timeout The maximum time to wait for a message, in microsecond.
 * @return int Returns 0 on success, - 1 on timeout, -2 on error message.
 */
int receiveCANMessage(CANMessage *msg, int timeout);
/**
 * @brief Unpacks a CAN message from raw data to a CANMessage structure.
 *
 * @param msg Pointer to a CANMessage structure where the unpacked message will be stored.
 * @param data Pointer to the raw data received from the CAN bus.
 */
void unpackCANMessage(CANMessage *msg, uint8_t *data);
void clearCanFIFO();
int checkErrorMsg(CANMessage *msg);

/**
 * @brief Saves the feedback from a CAN message to the specified motor. And updates motor's status.
 *
 * @param msg Pointer to the CANMessage containing the feedback data.
 * @param motor Pointer to the Motor object that will be updated with the feedback.
 */
void saveFeedback(CANMessage *msg, Motor *motor);
/**
 * @brief Sends a CAN message.
 *
 * @param data Pointer to the raw data to be sent.
 * @return HAL_StatusTypeDef Returns HAL_OK if the message is sent successfully, otherwise returns an error code.
 */
HAL_StatusTypeDef sendCANMessage(int id, uint8_t *data);

#endif /* CAN_H */
