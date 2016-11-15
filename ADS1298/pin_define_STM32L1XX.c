#include "include.h"
#include "pin_define_STM32L1XX.h"

// void AD1298_IOInit_STM32L1XX()
// {
// 	GPIO_InitTypeDef GPIO_InitStruct;

//   /* GPIO Ports Clock Enable */
//   __GPIOA_CLK_ENABLE();
//   __GPIOB_CLK_ENABLE();

//   /*Configure GPIO pins : PB0 PB1 PB2 PB4 
//                            PB6 PB7 PB8 */
//   GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_4 
//                           |GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8;
//   GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
//   GPIO_InitStruct.Pull = GPIO_PULLUP;
//   GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
//   HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

//   /*Configure GPIO pins : PB10 PB5 */
//   GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_5;
//   GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
//   GPIO_InitStruct.Pull = GPIO_PULLUP;
//   HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

//   /*Configure GPIO pins : PB3 PB9 */
//   GPIO_InitStruct.Pin = GPIO_PIN_3|GPIO_PIN_9;
//   GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
//   GPIO_InitStruct.Pull = GPIO_PULLDOWN;
//   GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
//   HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
// }

void delay_ms(uint ms)
{
	volatile uchar i,j;
	volatile uint n;
	for(i=0;i<100;i++)
		for(j=0;j<10;j++)
			for(n=ms;n>0;n--);
}
void delay_n(uint n)
{
    for(;n>0;n--)
        ;
}
