#ifndef _HARDWARE_H_
#define _HARDWARE_H_
#include "main.h"
#include "oled.h"

#define DAin HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_6)
#define CKOut_High HAL_GPIO_WritePin(GPIOA,GPIO_PIN_5,GPIO_PIN_SET)
#define CKOut_Low HAL_GPIO_WritePin(GPIOA,GPIO_PIN_5,GPIO_PIN_RESET)
#define LDOut_High HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4,GPIO_PIN_SET)
#define LDOut_Low HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4,GPIO_PIN_RESET)

void oled_pid_para_dis(int pid_No);

#endif
