#ifndef UTILITY_H
#define UTILITY_H

#include "mainpp.h"

#define PI 3.14159265358979323846
#define deg2rad(degree) (float)((degree) * PI / 180.0)
#define rad2degree(rad) (float)((rad) * 180.0 / PI)

class Clock
{
public:
    Clock(TIM_HandleTypeDef *htim);
    void start();
    int time();
private:
    TIM_HandleTypeDef *htim;
};

extern Clock clock1us;
extern Clock clock01ms;

#endif // UTILITY_H