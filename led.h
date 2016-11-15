#ifndef _LED_H_
#define _LED_H_

#define LEDGreen_PIN GPIO_PIN_14
#define LEDRed_PIN GPIO_PIN_13
#define LED_COM GPIOB

#define LEDGreen_SET LED_COM->BSRR = (uint32_t)LEDGreen_PIN
#define LEDGreen_CLEAR LED_COM->BSRR = (uint32_t)LEDGreen_PIN << 16

#define LEDRed_SET LED_COM->BSRR = (uint32_t)LEDRed_PIN
#define LEDRed_CLEAR LED_COM->BSRR = (uint32_t)LEDRed_PIN << 16

#define LEDGreen_Is ((LED_COM->ODR)& LEDGreen_PIN)? GPIO_PIN_SET:GPIO_PIN_RESET
#define LEDRed_Is ((LED_COM->ODR)& LEDRed_PIN)? GPIO_PIN_SET:GPIO_PIN_RESET

#endif 
