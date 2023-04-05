#ifndef __CONNECT_H__
#define __CONNECT_H__
#include "main.h"
#include "usart.h"
#include <stdio.h>

extern uint8_t  RecieveBuffer[1];
extern uint8_t usart2_buf[9];
extern uint8_t RxLen2;
extern uint8_t flag2;

extern uint8_t  RecieveBuffer3[1];
extern uint8_t usart3_buf[7];
extern uint8_t RxLen3;
extern uint8_t flag3;

extern float stepper_usart_angle[2];


#endif
