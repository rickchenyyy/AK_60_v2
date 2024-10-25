#include "utility.h"

Clock clock1us(&htim1); // 16bits, max 65535us
Clock clock01ms(&htim2); // 32bits, max 4294967 * 0.1ms = 429496.7s


Clock::Clock(TIM_HandleTypeDef *htim) : htim(htim)
{}

void Clock::start()
{
    __HAL_TIM_SET_COUNTER(htim, 0);
}

int Clock::time()
{
    return __HAL_TIM_GET_COUNTER(htim);
}