//***************************************************************************//
//                                                                           //
// �ļ���  pin_define.h                                                      //
// ˵����  �ĵ��¼������������Ƭ�����Ź��ܶ�����IO�����궨��                //
// ���룺  keil 4  STM32L151C8T6                      //
// �汾��  v1.0                                                              //
// ��д��  ��Դ@2015-10 ����                                //                               //
//                                                                           //
//***************************************************************************//

#ifndef  _PIN_DEFINE_H_
#define  _PIN_DEFINE_H_

	/*
Pin Nb		PINs		FUNCTIONs	LABELs
15	PA5		SPI1_SCK	AD_SCLK
16	PA6		SPI1_MISO	AD_DIN
17	PA7		SPI1_MOSI	AD_DOUT

18	PB0		GPIO_Output	AD_START
19	PB1		GPIO_Output	AD_RESET
20	PB2		GPIO_Output	AD_PWDN
39	PB3		GPIO_Output	AD_CS
40	PB4		GPIO_Output	AD_CLKSEL
41	PB5		GPIO_Input	AD_DRDY

42	PB6		GPIO_Output	AD_RESET_2
43	PB7		GPIO_Output	AD_PWDN_2
45	PB8		GPIO_Output	AD_CS_2
46	PB9		GPIO_Output	AD_CLKSEL_2
21	PB10	GPIO_Input	AD_DRDY_2

30	PA9		USART1_TX	
31	PA10	USART1_RX	
	
	*/




//*********************************���ڶ˿ڶ���******************************//

	

//****************************ADS1298���Ź��ܶ���****************************//



#define AD_START                GPIO_PIN_0 //ADS1298 ��ʼ�ɼ��ź� PB0

/*************************************�ǹ��ùܽ�**********************************************/
//��һ��
#define AD_RESET                GPIO_PIN_1 //ADS1298 ��λ�ź�     PB1
#define AD_PWDN                 GPIO_PIN_10 //ADS1298 �����ź�     PB10
#define AD_CS                   GPIO_PIN_11 //ADS1298 Ƭѡ�ź�     PB11
#define AD_CLKSEL               GPIO_PIN_12 // ADS1298��ʱ��ѡ�����ţ�0���ڲ�ʱ�ӣ�1���ⲿʱ�� PB12
#define AD_DRDY                 GPIO_PIN_5 //ADS1298 ����ת������ź�   PB5


//SPI���Ŷ���
#define AD_SCLK                 GPIO_PIN_5 //ADS1298 ͨ��ʱ���ź�   PA5
#define AD_DOUT                 GPIO_PIN_6 //ADS1298 ���������ź�   PA6
#define AD_DIN                  GPIO_PIN_7 //ADS1298 ��������ź�   PA7


//�ڶ���

#define AD_RESET_2      	    GPIO_PIN_6       //PB6
#define AD_PWDN_2       	    GPIO_PIN_7       //PB7
#define AD_CS_2         	    GPIO_PIN_8       //PB8
#define AD_CLKSEL_2     	    GPIO_PIN_9       //PB9
#define AD_DRDY_2        		  GPIO_PIN_0       //PA0


//*********************�ܽ�COM����*************************************************//
#define AD_START_COM                GPIOB


#define AD_RESET_COM                 GPIOB //ADS1298 ��λ�ź�     PB1
#define AD_PWDN_COM                  GPIOB //ADS1298 �����ź�     PB10
#define AD_CS_COM                    GPIOB //ADS1298 Ƭѡ�ź�     PB11
#define AD_CLKSEL_COM                GPIOB // ADS1298��ʱ��ѡ�����ţ�0���ڲ�ʱ�ӣ�1���ⲿʱ�� PB12
#define AD_DRDY_COM                  GPIOB //ADS1298 ����ת������ź�   PB5


//SPI���Ŷ���
#define AD_SCLK_COM                  GPIOA //ADS1298 ͨ��ʱ���ź�   PA5
#define AD_DIN_COM                   GPIOA //ADS1298 ���������ź�   PA6
#define AD_DOUT_COM                  GPIOA //ADS1298 ��������ź�   PA7



//�ڶ���
#define AD_RESET_2_COM       	    GPIOB       //PB6
#define AD_PWDN_2_COM        	    GPIOB       //PB7
#define AD_CS_2_COM          	    GPIOB       //PB8
#define AD_CLKSEL_2_COM      	    GPIOB       //PB9
#define AD_DRDY_2_COM         		GPIOA       //PA0





//***************************************************************************//
//                                                                           
//                           ADS1298�����궨��                                 
//                                                                           
//***************************************************************************//

//��һ��ads1298����
#define  SET_ADS1298_START       AD_START_COM->BSRR = (uint32_t)AD_START   //ADS1298��ʼ�ɼ�
#define  CLR_ADS1298_START       AD_START_COM->BSRR = (uint32_t)AD_START << 16 
#define  SET_ADS1298_RST         AD_RESET_COM->BSRR = (uint32_t)AD_RESET   //ADS1298��λ
#define  CLR_ADS1298_RST         AD_RESET_COM->BSRR = (uint32_t)AD_RESET << 16 
#define  SET_ADS1298_PWDN        AD_PWDN_COM->BSRR = (uint32_t)AD_PWDN    //ADS1298����ģʽ
#define  CLR_ADS1298_PWDN        AD_PWDN_COM->BSRR = (uint32_t)AD_PWDN << 16 
#define  SET_ADS1298_CS          AD_CS_COM->BSRR = (uint32_t)AD_CS     //ADS1298Ƭѡ�ź�
#define  CLR_ADS1298_CS          AD_CS_COM->BSRR = (uint32_t)AD_CS << 16 
#define  ADS1298_READY_FLAG      HAL_GPIO_ReadPin(AD_DRDY_COM,AD_DRDY)			//�����ţ����ȼ�����

#define  ADS1298_CLOCK_INTER    AD_CLKSEL_COM->BSRR = (uint32_t)AD_CLKSEL  //������ʱ��ѡ��
#define  ADS1298_CLOCK_EXTER    AD_CLKSEL_COM->BSRR = (uint32_t)AD_CLKSEL << 16 

//�ڶ���ads1298����

#define  SET_ADS1298_RST_2         AD_RESET_2_COM->BSRR = (uint32_t)AD_RESET_2   //ADS1298_2��λ
#define  CLR_ADS1298_RST_2         AD_RESET_2_COM->BSRR = (uint32_t)AD_RESET_2 << 16 
#define  SET_ADS1298_PWDN_2        AD_PWDN_2_COM->BSRR = (uint32_t)AD_PWDN_2    //ADS1298_2����ģʽ
#define  CLR_ADS1298_PWDN_2        AD_PWDN_2_COM->BSRR = (uint32_t)AD_PWDN_2 << 16 
#define  SET_ADS1298_CS_2          AD_CS_2_COM->BSRR = (uint32_t)AD_CS_2      //ADS1298_2Ƭѡ�ź�
#define  CLR_ADS1298_CS_2          AD_CS_2_COM->BSRR = (uint32_t)AD_CS_2 << 16 
#define  ADS1298_READY_FLAG_2      HAL_GPIO_ReadPin(AD_DRDY_2_COM,AD_DRDY_2)       //�����ţ����ȼ�����

#define  ADS1298_CLOCK_INTER_2    AD_CLKSEL_2_COM->BSRR = (uint32_t)AD_CLKSEL_2   //������ʱ��ѡ��
#define  ADS1298_CLOCK_EXTER_2    AD_CLKSEL_2_COM->BSRR = (uint32_t)AD_CLKSEL_2 << 16 

/*
//����ͨ�Źܽ�
#define  SET_ADS128_SCLK        P3OUT |=  AD_SCLK    //ADS1298 SPIͨ��SCLK
#define  CLR_ADS128_SCLK        P3OUT &=~ AD_SCLK
#define  SET_ADS128_DIN         P3OUT |=  AD_DIN    //ADS1298 SPIͨ��DIN
#define  CLR_ADS128_DIN         P3OUT &=~ AD_DIN
#define  ADS1298_DOUT_DATA      (P3IN & AD_DOUT)       //ADS1298 SPIͨ��DIN

*/

void delay_ms(unsigned int ms);
void delay_n(unsigned int  n);

//void IO_Init_STM32L1XX();

#endif



//*******************************end file************************************//

