#ifndef MAINPP_H
#define MAINPP_H


// Assuming Motor class is defined in a header file

#ifdef __cplusplus
extern "C" {
#endif

// includes here will link to main.c (C compiler)
#include <stdio.h>
#include <string.h>
#include "main.h"


void mainpp();
int goHome();
int Control1khz();



extern char g_status[30];
extern CAN_TxHeaderTypeDef TxHeader;
extern CAN_RxHeaderTypeDef RxHeader;


#ifdef __cplusplus
}
#endif

#endif // MAINPP_H
