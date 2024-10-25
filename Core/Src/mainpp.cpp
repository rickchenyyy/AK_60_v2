#include "mainpp.h"
#include "config.h"
#include "can.h"
#include "utility.h"
#include "stm32f4xx_hal.h"

char g_status[30] = "start";

// CAN header
CAN_TxHeaderTypeDef TxHeader;
CAN_RxHeaderTypeDef RxHeader;

CANMessage msg;

void mainpp()
{
    int ret = 0;
    printf("Hello from mainpp\n");

    canInit();

START_MOTOR:
    strcpy(g_status, "start_motor");
    ret = startMotor();
    if (ret == -1)
    {
        printf("Error in startMotor\n");
        strcpy(g_status, "wait_command2");
    }
    else
    {
        printf("Motor started\n");
        HAL_Delay(2000);
        strcpy(g_status, "go_home");
        // motors[0].setPosCommand(0.0);
        // strcpy(g_status, "running_control");
    }
    while (1)
    {
        if (strcmp(g_status, "wait_command1") == 0)
        {
            if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13) == GPIO_PIN_RESET)
            {
                strcpy(g_status, "running_control");
            }
        }
        else if (strcmp(g_status, "wait_command2") == 0)
        {
            if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13) == GPIO_PIN_RESET)
            {
                goto START_MOTOR;
            }
            HAL_Delay(100);
        }
        printf("In mainpp, status=%s\n", g_status);
        HAL_Delay(1000);
    }
    return;
}

int Control1khz()
{
    int ret = 0;
    static int tick = 0;
    for (int i = 0; i < NUM_MOTORS; i++)
    {
        clock01ms.start();
        motors[i].setTorqueCommand(motors[i].controllers[0].torqueCommand(&motors[i]));
        if (sendCANMessage(motors[i].id, motors[i].Torque_cmd_tx) == HAL_OK)
        {
        }
        else
        {
            printf("Error in Control1khz(): Error Sending CAN\n");
            return -1;
        }
        ret = receiveCANMessage(&msg, 20);
        if (ret == 0)
        {
            saveFeedback(&msg, &motors[i]);
            if (tick % 100 == 0)
            {
                printf("%d, %d, %.3f, %.3f, %.3f, %.3f, %d; ", tick, motors[i].id, motors[i].pos_command, motors[i].pos_feedback, motors[i].torque_command, motors[i].torque_feedback, clock01ms.time());
            }
        }
        else if (ret == -1)
        {
            printf("Error in Control1khz(): timeout\n");
            return -1;
        }
        else if (ret == -2)
        {
            printf("Error in Control1khz(): error message\n");
            return -1;
        }
        if (motors[i].checkStatus() == -2)
        {
            return -1;
        }
    }
    if (tick % 100 == 0)
    {
        printf("\n");
    }
    tick++;
    return 0;
}

int goHome()
{
    // TODO: Implement goHome
    int ret = 0;
    static int tick = 0;
    double pos_err = 0;
    for (int i = 0; i < NUM_MOTORS; i++)
    {
        pos_err = motors[i].pos_feedback - motors[i].home;
        if (pos_err < motors[i].gohome_tolerance[0] || pos_err > motors[i].gohome_tolerance[1])
        {
            motors[i].setTorqueCommand(0);
            motors[i].pos_command = motors[i].pos_feedback;
            if (tick % 500 == 0){
            	printf("Motor %d is out of tolerance\n", motors[i].id);
            }
        }
        else
        {
            if (fabs(pos_err) < deg2rad(5))
            {
                motors[i].is_home = true;
            }
            motors[i].setPosCommand(motors[i].home);
            motors[i].setTorqueCommand(motors[i].controllers[0].torqueCommand(&motors[i]));
            if (tick % 500 == 0)
            {
                printf("In goHome(),Motor %d,home: %.3f,pos: %.3f,pos_err: %.3f\n", i, motors[i].home, motors[i].pos_feedback, pos_err);
            }
        }

        // Start sending CAN message
        if (sendCANMessage(motors[i].id, motors[i].Torque_cmd_tx) == HAL_OK)
        {
        }
        else
        {
            printf("Error in goHome(): Error Sending CAN\n");
            return -1;
        }
        ret = receiveCANMessage(&msg, 20);
        if (ret == 0)
        {
            saveFeedback(&msg, &motors[i]);
        }
        else if (ret == -1)
        {
            printf("Error in goHome(): timeout\n");
            return -1;
        }
        else if (ret == -2)
        {
            printf("Error in goHome(): error message\n");
            return -1;
        }
        if (motors[i].checkStatus() == -2)
        {
            return -1;
        }
    }
    tick++;
    return 0;
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    //     printf("TIM3 callback time = %d (0.1ms)\n\r", clock01ms.time());
    //     clock01ms.start();
    //    printf("TIM3 callback time = %d us\n\r", clock1us.time());
    //    clock1us.start();
    if (htim == &htim3)
    {
        static int i = 0;
        int ret = 0;
        // printf("In 1kHz callback, i=%d\n", i++);
        if (strcmp(g_status, "running_control") == 0)
        {
            ret = Control1khz();
        }
        else if ((strcmp(g_status, "go_home") == 0) || (strcmp(g_status, "wait_command1") == 0))
        {
            ret = goHome();
            if (allAtHome() && (strcmp(g_status, "go_home") == 0))
            {
                strcpy(g_status, "wait_command1");
            }
        }
        else
        {
        }
        if (ret == -1)
        {
            printf("status: %s, ret: %d, stop motor\n", g_status, ret);
            stopMotor();
            strcpy(g_status, "wait_command2");
        }
    }
}
