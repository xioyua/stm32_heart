/*coutain the all inc file*/
#ifndef _INCLUDE_H
#define _INCLUDE_H


#include "stm32l1xx_hal.h"
#include "stm32l1xx.h"
#include "TASK_SPI1.H"
#include "TASK_USART1.H"
#include "ADS1298_STM32L1XX.H"
#include "pin_define_STM32L1XX.h"
#include "BlueTooth.h"
#include "led.h"
#include "key.h"

typedef uint8_t uchar;
typedef unsigned int  uint;

extern UART_HandleTypeDef huart1;
extern SPI_HandleTypeDef hspi1;

extern uint16_t TIM2Interrupt;
extern uint16_t TIM3Interrupt;

extern uint8_t gBlueToothSetFlag;
#endif


