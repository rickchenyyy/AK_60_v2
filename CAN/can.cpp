#include "can.h"

uint8_t RxData[8];
uint32_t TxMailbox;

void canInit()
{
    HAL_CAN_Start(&hcan1);
    TxHeader.DLC = 8; // data length
    TxHeader.IDE = CAN_ID_STD;
    TxHeader.RTR = CAN_RTR_DATA;

    // Activate the notification
    //    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);

    CAN_FilterTypeDef canfilterconfig;

    canfilterconfig.FilterActivation = CAN_FILTER_ENABLE;
    canfilterconfig.FilterBank = 18;
    canfilterconfig.FilterFIFOAssignment = CAN_FILTER_FIFO0;
    canfilterconfig.FilterIdHigh = 0x000 << 5; // Identifier
    canfilterconfig.FilterIdLow = 0;
    canfilterconfig.FilterMaskIdHigh = 0x000 << 5;
    canfilterconfig.FilterMaskIdLow = 0x0000;
    canfilterconfig.FilterMode = CAN_FILTERMODE_IDMASK;
    canfilterconfig.FilterScale = CAN_FILTERSCALE_32BIT;
    canfilterconfig.SlaveStartFilterBank = 20;

    HAL_CAN_ConfigFilter(&hcan1, &canfilterconfig);
}

uint8_t Start_Motor[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC};
uint8_t Stop_Motor[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFD};
int startMotor()
{
    printf("In startMotor, wait 1s\n");
    stopMotor();
    HAL_Delay(1000);
    CANMessage msg;
    int ret = 0;
    for (int i = 0; i < NUM_MOTORS; i++)
    {
        TxHeader.StdId = motors[i].id;
        clock01ms.start();
        while (1)
        {
            ret = sendCANMessage(motors[i].id, Start_Motor);
            if (ret == HAL_OK)
            {
            }
            else
            {
                printf("Motor %d start Error ret=%d\n", i, ret);
                return -1;
            }
            if (clock01ms.time() > 30000)
            { // 3s
                printf("Motor %d start timeout\n", i);
                return -1;
            }
            ret = receiveCANMessage(&msg, 100);
            if (ret == 0)
            {
                break;
            }
            else if (ret == -1)
            {
                continue;
            }
            else if (ret == -2)
            {
                return -1;
            }
        }
        HAL_Delay(100);
    }
    HAL_Delay(100);
    // Double check if all motors are started and can loop-time within certain time
    int min_loop_time_us = 50;
    for (int i = 0; i < NUM_MOTORS; i++)
    {
        clock1us.start();
        ret = sendCANMessage(motors[i].id, Start_Motor);
        if (ret == HAL_OK)
        {
        }
        else
        {
            printf("Motor %d start Error ret=%d\n", i, ret);
            return -1;
        }
        ret = receiveCANMessage(&msg, min_loop_time_us);
        saveFeedback(&msg, &motors[i]);
        if (ret == 0)
        {
            printf("Motor %d started, loop-time = %d us\n", i, clock1us.time());
            motors[i].pos_command = motors[i].pos_feedback;
        }
        else if (ret == -1)
        {
            continue;
        }
        else if (ret == -2)
        {
            return -1;
        }
        HAL_Delay(50);
    }
    return 0;
}

HAL_StatusTypeDef sendCANMessage(int id, uint8_t *data)
{
    TxHeader.StdId = id;
    HAL_StatusTypeDef ret = HAL_CAN_AddTxMessage(&hcan1, &TxHeader, data, &TxMailbox);
    return ret;
}

void stopMotor()
{
    for (int i = 0; i < NUM_MOTORS; i++)
    {
        if (sendCANMessage(motors[i].id, Stop_Motor) == HAL_OK)
        {
        }
        else
        {
            printf("Error2 in stopMotor......\n");
            return;
        }
    }
}

int receiveCANMessage(CANMessage *msg, int timeout)
{
    CAN_RxHeaderTypeDef RxHeader;
    uint8_t RxData[8];
    HAL_StatusTypeDef status;

    // Wait for a message with a timeout
    clock1us.start();
    while (clock1us.time() < timeout)
    {
        if (HAL_CAN_GetRxFifoFillLevel(&hcan1, CAN_RX_FIFO0) > 0)
        {
            status = HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &RxHeader, RxData);
            if (status == HAL_OK)
            {
                // Process the received message
                unpackCANMessage(msg, RxData);
                if (checkErrorMsg(msg) == -1)
                {
                    return -2;
                }
                return 0;
            }
        }
    }
    // Timeout occurred
    // printf("receiveCANMessage: Timeout\n");
    return -1;
}

void unpackCANMessage(CANMessage *msg, uint8_t *data)
{
    // Initialize variables
    static int id = 0, p_int = 0, v_int = 0, t_int = 0;
    static float pos = 0.0, vel = 0.0, tor = 0.0;

    // Unpack the data
    id = data[0];
    p_int = (data[1] << 8) | data[2];         // Motor position data
    v_int = (data[3] << 4) | (data[4] >> 4);  // Motor speed data
    t_int = ((data[4] & 0xF) << 8) | data[5]; // Motor torque data

    // Convert integers to floats
    pos = INT_TO_FLOAT_POS(p_int);
    vel = INT_TO_FLOAT_VEL(v_int);
    tor = INT_TO_FLOAT_TORQUE(t_int);

    // Put data into the CANMessage structure
    msg->id = id;
    msg->pos = pos;
    msg->vel = vel;
    msg->torque = tor;
    msg->error_code = 0; // **TODO**: check error code
}

// **TODO**: check error code
int checkErrorMsg(CANMessage *msg)
{
    if (msg->error_code != 0)
    {
        printf("Error code: %d\n", msg->error_code);
        return -1;
    }
    return 0;
}

void clearCanFIFO()
{
    CAN_RxHeaderTypeDef RxHeader;
    uint8_t RxData[8];
    CANMessage msg;

    while (HAL_CAN_GetRxFifoFillLevel(&hcan1, CAN_RX_FIFO0) > 0)
    {
        receiveCANMessage(&msg, 100);
    }
}

void saveFeedback(CANMessage *msg, Motor *motor)
{
    if (msg->id == motor->id)
    {
        motor->pos_feedback = msg->pos;
        motor->velocity_feedback = msg->vel;
        motor->torque_feedback = msg->torque;
        if (motor->pos_feedback < motor->warning_pos[0] || motor->pos_feedback > motor->warning_pos[1])
        {
            motor->is_warning = true;
        }
        else
        {
            motor->is_warning = false;
        }
        if (motor->pos_feedback < motor->fatal_pos[0] || motor->pos_feedback > motor->fatal_pos[1])
        {
            motor->is_fatal = true;
        }
        else
        {
            motor->is_fatal = false;
        }
    }
    else
    {
        printf("Error in saveFeedback: id mismatch\n");
    }
}