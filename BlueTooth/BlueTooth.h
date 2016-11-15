#ifndef _BLUETOOTH_H_
#define _BLUETOOTH_H_

#define BS_Key_Pin  GPIO_PIN_14
#define BS_Key_COM	GPIOC

#define  SET_BS_Key       BS_Key_COM->BSRR = (uint32_t)BS_Key_Pin   //ADS1298开始采集
#define  CLR_BS_Key       BS_Key_COM->BSRR = (uint32_t)BS_Key_Pin << 16 


typedef enum blueFlag
{
    BLUE_OK = 0,
    BLUE_ERROR = 1,
    BLUE_NONE = 2,
    BLUE_ERROR2,
    BLUE_ERROR3,
    BLUE_ERROR4,
    BLUE_ERROR5,
    BLUE_ERROR6
} typeBlueFlag;
    


void BuleTooth_Init(void);

#endif

