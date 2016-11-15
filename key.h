#ifndef _KEY_H_
#define _KEY_H_

#define KEY_PIN GPIO_PIN_13
#define KEY_COM GPIOC


#define Key_Is ((KEY_COM->IDR)& KEY_PIN)? GPIO_PIN_SET:GPIO_PIN_RESET


#endif 
