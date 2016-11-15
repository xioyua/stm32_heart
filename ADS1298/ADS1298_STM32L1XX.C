//===========================================================================//
//                                                                           //
// 文件：  ADS1298.C                                                          //
// 说明：  心电记录仪模数转换器ADS1298控制功能部分                           //
// 编译：  IAR Embedded Workbench IDE for msp430 v4.21                       //
// 版本：  v1.0                                                              //
// 编写：  秦佩@地质宫306实验室，2011-11-2                                  //
// 版权：  吉林大学朝阳校区仪电学院弱磁实验室                                //
//                                                                           //
//===========================================================================//

//头文件位置
#include "include.h"

//#include "intrinsics.h"
//#include "serial.h"
//#include <stdio.h>

/**************************************************************************************************************************************************
*                                 Global Variables                                                                                                *
**************************************************************************************************************************************************/
unsigned char ADS1x9x_SPI_data;

// volatile register_name_type *register_name = (volatile register_name_type*)0xADD;
volatile unsigned char ADS1x9x_Config_1;
volatile ADS1x9x_Config_1_Register_type *ADS1x9x_Config_1_register = (volatile ADS1x9x_Config_1_Register_type*) &ADS1x9x_Config_1;
volatile unsigned char ADS1x9x_Config_2;
volatile ADS1x9x_Config_2_Register_type *ADS1x9x_Config_2_register = (volatile ADS1x9x_Config_2_Register_type*) &ADS1x9x_Config_2;
volatile unsigned char ADS1x9x_Config_3;
volatile ADS1x9x_Config_3_Register_type *ADS1x9x_Config_3_register = (volatile ADS1x9x_Config_3_Register_type*) &ADS1x9x_Config_3;
volatile unsigned char ADS1x9x_Config_4;
volatile ADS1x9x_Config_4_Register_type *ADS1x9x_Config_4_register = (volatile ADS1x9x_Config_4_Register_type*) &ADS1x9x_Config_4;
volatile unsigned char ADS1x9x_Default_Register_Watch[23];
volatile unsigned char ADS1x9x_Default_Register_Watch2[23];
unsigned char ADS1x9x_Default_Register_Settings [23] =
{
  0x64, // Config1
  0x31, // Config2
  0xCC, // Config3,使用内部参考电压
  0x03, // loff
  0x40, // ch1set
  0x40, // ch2set
  0x40, // ch3set
  0x40, // ch4set
  0x40, // ch5set
  0x40, // ch6set
  0x40, // ch7set
  0x40, // ch8set
  0x00, // rld_sensp
  0xf9, // rld_sensn，使用所有通道的N端作为右腿驱动的源
  0xff, // loff_sensp
  0xff, // loff_sensm
  0x00, // loff_flip
  0x00, // gpio
  0x00, // resp
  0x01, // pace
  0x02, // config4,bit3:one-shot;bit2:wct=?rld;bit1:led-off
  0x0B, // wct1 0x0B
  0xD4  // wct2 0xD4
};

unsigned char ADS1x9x_Default_Register_Settings_2 [23] =
{
  0x44, // Config1 default 0x86
  0x31, // Config2
  0xCC, // Config3,使用内部参考电压
  0x03, // loff
  0x80, // ch1set
  0x80, // ch2set
  0x40, // ch3set
  0x40, // ch4set
  0x40, // ch5set
  0x40, // ch6set
  0x40, // ch7set
  0x40, // ch8set
  0x00, // rld_sensp
  0x00, // rld_sensm，使用所有通道的N端作为右腿驱动的源
  0xff, // loff_sensp
  0xff, // loff_sensm
  0x00, // loff_flip
  0x00, // gpio
  0x00, // resp
  0x00, // pace
  0x02, // config4,bit3:one-shot;bit2:wct=?rld;bit1:led-off
  0x00, // wct1 0X0A
  0x00  // wct2 0XE3
};

const unsigned char *ADS1x9x_download_pointer = ADS1x9x_Default_Register_Settings;
const unsigned char *ADS1x9x_download_pointer_2 = ADS1x9x_Default_Register_Settings_2;
volatile unsigned char ADS1x9x_Lead_Off_Control;
volatile ADS1x9x_Lead_Off_Control_Register_type *ADS1x9x_Lead_Off_Control_Register = (volatile ADS1x9x_Lead_Off_Control_Register_type*)&ADS1x9x_Lead_Off_Control;

volatile unsigned char ADS1x9x_Channel_Settings;
volatile ADS1x9x_Channel_Settings_Register_type *ADS1x9x_Channel_Settings_Register = (volatile ADS1x9x_Channel_Settings_Register_type*)&ADS1x9x_Channel_Settings;

volatile unsigned char ADS_1298_Channel_Stack [8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

//unsigned char ADS1x9x_SPI_Settings [6] = {LOW_POLARITY, RETARDED_DATA, _8MHZ_MAX_CLOCK, AFE_ADS1x9x, 0x00, SLOT_NUMBER_1};

volatile unsigned char ADS1x9x_SPI_TX_BUFFER [4];
volatile unsigned char ADS1x9x_SPI_RX_BUFFER [4];
//volatile USCI_Interrupt_Flag_Status_type* ADS1x9x_SPI_Interrupt_Flags[4];

volatile unsigned char ADS1x9x_Version_ID_Number,ADS1x9x_Version_ID_Number_2;

volatile ADS1x9x_Status_Flags_type ADS1x9x_Status_Flags = {0x00,0x00,0x00,0x00};


/**************************************************************************************************************************************************
*    Initialize the ADS1x9x device  for work mode 3                                                                                             *
**************************************************************************************************************************************************/
unsigned char init_ADS1x9x3 ()
{
  volatile unsigned char Verify_Check = CLEAR;
  unsigned char Module_Present = CLEAR;
  
  unsigned char number_of_retries = 10;
  //unsigned char i;
  volatile unsigned char version =0;
	
  //init_ADS1x9x_Data_Ready_Interrupt ();                            // Set up the ADS1x9x Interrupt pin
  //enable_ADS1x9x_Interrupt ();                                     // Interrupts required for SPI communication

  //while(1)   //调试ADS1298的连接IO初始化是否正确
  init_ADS1x9x_IO ();

  Power_Up_ADS1x9x ();                                             // Power up Digital portion of the ADS1x9x
  delay_ms(4000);
  POR_Reset_ADS1x9x ();

	
  Stop_Read_Data_Continuous();
	

                                                                                         
//	Set_ADS1x9x_Chip_Enable_2();
//	Clear_ADS1x9x_Chip_Enable();
//	spi_sendChar(0x20);
//	spi_sendChar(0x00);
//	for(i = 0; i<5; i++)
//		version = spi_readChar(SPI_TEST_DATA);
// ADS1x9x_SPI_Data(0x20);
// ADS1x9x_SPI_Data(0x00);
// version = ADS1x9x_SPI_Data(SPI_TEST_DATA);
 // delay_ms(10);
 // while(1)  //调试ADS1298和单片机之间的SPI通信用  
  CLR_ADS1298_PWDN_2;
	
// 	for(i=1;i<0x20;i++)
// 	{
// 		if(i<0x12)
// 		{
// 			version = setAndGetRegister(i, ADS1x9x_download_pointer[i-1]);
// 		}
// 		else if(i>0x13)
// 		{
// 			version = setAndGetRegister(i, ADS1x9x_download_pointer[i-3]);
// 		}
// 	}
// 	version = setAndGetRegister(0x01, 0x66);
// 	Verify_Check = verify_ADS1x9x_Registers ((unsigned char*) ADS1x9x_download_pointer);
//   version = setAndGetRegister(0x02, 0x10);
//   version = setAndGetRegister(0x03, 0xc4);
// 	Verify_Check = verify_ADS1x9x_Registers ((unsigned char*) ADS1x9x_download_pointer);
//   version = setAndGetRegister(0x04, 0x03);
// 	version = setAndGetRegister(0x05, 0x00);
// 	Verify_Check = verify_ADS1x9x_Registers ((unsigned char*) ADS1x9x_download_pointer);
// 	version = setAndGetRegister(0x06, 0x00);
// 	version = setAndGetRegister(0x07, 0x00);
// 	Verify_Check = verify_ADS1x9x_Registers ((unsigned char*) ADS1x9x_download_pointer);
// 	version = setAndGetRegister(0x08, 0x00);
// 	version = setAndGetRegister(0x09, 0x00);
// 	Verify_Check = verify_ADS1x9x_Registers ((unsigned char*) ADS1x9x_download_pointer);
// 	version = setAndGetRegister(0x0A, 0x00);
// 	version = setAndGetRegister(0x0B, 0x00);
// 	Verify_Check = verify_ADS1x9x_Registers ((unsigned char*) ADS1x9x_download_pointer);
// 	version = setAndGetRegister(0x0C, 0x00);
// 	version = setAndGetRegister(0x0D, 0xFF);
// 	Verify_Check = verify_ADS1x9x_Registers ((unsigned char*) ADS1x9x_download_pointer);
// 	version = setAndGetRegister(0x0E, 0xFF);
// 	version = setAndGetRegister(0x0F, 0xFF);
// 	Verify_Check = verify_ADS1x9x_Registers ((unsigned char*) ADS1x9x_download_pointer);
// 	version = setAndGetRegister(0x10, 0xFF);
// 	version = setAndGetRegister(0x11, 0x00);
// 	Verify_Check = verify_ADS1x9x_Registers ((unsigned char*) ADS1x9x_download_pointer);
// 	version = setAndGetRegister(0x14, 0x00);
// 	version = setAndGetRegister(0x15, 0x00);
// 	Verify_Check = verify_ADS1x9x_Registers ((unsigned char*) ADS1x9x_download_pointer);
// 	version = setAndGetRegister(0x16, 0x00);
// 	Verify_Check = verify_ADS1x9x_Registers ((unsigned char*) ADS1x9x_download_pointer);
// 	version = setAndGetRegister(0x17, 0x02);
// 	Verify_Check = verify_ADS1x9x_Registers ((unsigned char*) ADS1x9x_download_pointer);
// 	version = setAndGetRegister(0x18, 0x0B);
// 	version = setAndGetRegister(0x19, 0xD4);
	
  init_ADS1x9x_Via_Constant_Table ((unsigned char*) ADS1x9x_download_pointer);
  Verify_Check = verify_ADS1x9x_Registers ((unsigned char*) ADS1x9x_download_pointer);
  //Verify_Check = Initialize_ADS1x9x_Data_Rate (MODULATION_FREQUENCY_DIVIDED_BY_1024 );  // DEFAULT_MODULATION_FREQUENCY_DIVIDED_BY_16
                                                                                         // MODULATION_FREQUENCY_DIVIDED_BY_32  16Ksps/s
                                                                                         // MODULATION_FREQUENCY_DIVIDED_BY_64  8Ksps/s
                                                                                         // MODULATION_FREQUENCY_DIVIDED_BY_128 4Ksps/s
                                                                                         // MODULATION_FREQUENCY_DIVIDED_BY_256 2Ksps/s
                                                                                         // MODULATION_FREQUENCY_DIVIDED_BY_512
                                                                                         // MODULATION_FREQUENCY_DIVIDED_BY_1024
  
  SET_ADS1298_PWDN_2;
  ADS1x9x_SPI_Data_2(STOP_READ_DATA_CONTINUOUSLY);                                                                                           
  ADS1x9x_Version_ID_Number_2 = ADS1x9x_Read_Version_2 ();
  ADS1x9x_Version_ID_Number = ADS1x9x_Read_Version ();
  while (!Module_Present  )                                                     // Wait for Module to be present
  {
    if (number_of_retries)
    {
      ADS1x9x_Version_ID_Number = ADS1x9x_Read_Version ();
      ADS1x9x_Version_ID_Number_2 = ADS1x9x_Read_Version_2 ();
      //Serial_Print_Char(ADS1x9x_Version_ID_Number);
      //Serial_Print_Char(ADS1x9x_Version_ID_Number_2);
      if (ADS1x9x_Version_ID_Number == 0x92 && ADS1x9x_Version_ID_Number_2 == 0x92)                  // (0x22 for old board, 0x42 for new one)
      {
        Module_Present = SET;
      }
      number_of_retries--;
    }
    else
    {

      return ADS_1x9x_NOT_FOUND;
    }
  }

  
  init_ADS1x9x_Via_Constant_Table_2 ((unsigned char*) ADS1x9x_download_pointer_2);

  
  Verify_Check = verify_ADS1x9x_Registers_2 ((unsigned char*) ADS1x9x_download_pointer_2);

  
  //Verify_Check = Initialize_ADS1x9x_Data_Rate_2 (MODULATION_FREQUENCY_DIVIDED_BY_1024 );

  //Verify_Check = Initialize_ADS1x9x_Mode (HIGH_RESOLUTION_MODE);                         // DEFAULT_LOW_POWER_MODE, HIGH_RESOLUTION_MODE
  //Verify_Check = Initialize_ADS1x9x_Mode_2 (HIGH_RESOLUTION_MODE);
  //用于测试单个通道工作时的时序操作
  //Initialize_ADS1x9x_Channel(1,DEFAULT_ADS1x9x_ELECTRODE_INPUT,GAIN_OF_2,ENABLE_POWER_DOWN,IGNORE_PREVIOUS_STATE);
  //for (i = 0; i < ECG_Num_Channels; i++)
  //{
  //  Verify_Check = Initialize_ADS1x9x_Channel                               //  Context Save will store the previous setting of the channel
  //    (

  //     i + 1,                                                              // references channels 1 - 8
  //     //ADS1x9x_ONE_HALF_DIGITAL_SUPPLY,
  //      //ADS1x9x_TEST_SIGNAL ,
  //     DEFAULT_ADS1x9x_ELECTRODE_INPUT,                                    // DEFAULT_ADS1x9x_ELECTRODE_INPUT, ADS1x9x_INPUT_SHORTED, ADS1x9x_RIGHT_LEG_DETECT, ADS1x9x_ONE_HALF_DIGITAL_SUPPLY
  //     //ADS1x9x_INPUT_SHORTED,                                            // ADS1x9x_TEMPERATURE_SENSOR, ADS1x9x_CALIBRATION_SIGNAL, ADS1x9x_RIGHT_LEG_DETECT_POSITIVE, ADS1x9x_RIGHT_LEG_DETECT_NEGATIVE
  //     //DEFAULT_GAIN_OF_6,                                                          // DEFAULT_GAIN_OF_6, GAIN_OF_1, GAIN_OF_2, GAIN_OF_3, GAIN_OF_4, GAIN_OF_8, GAIN_OF_12
  //     GAIN_OF_3,
  //     DEFAULT_DISABLE_POWER_DOWN,                                         // DEFAULT_DISABLE_POWER_DOWN, ENABLE_POWER_DOWN
  //     IGNORE_PREVIOUS_STATE                                               // CONTEXT_SAVE_CHANNEL, IGNORE_PREVIOUS_STATE
  //     );
  //  //Initialize_ADS1x9x_Channel(5,DEFAULT_ADS1x9x_ELECTRODE_INPUT,GAIN_OF_2,DEFAULT_DISABLE_POWER_DOWN,IGNORE_PREVIOUS_STATE);

  ////Initialize_ADS1x9x_Channel(6,DEFAULT_ADS1x9x_ELECTRODE_INPUT,GAIN_OF_2,DEFAULT_DISABLE_POWER_DOWN,IGNORE_PREVIOUS_STATE);

  ////Initialize_ADS1x9x_Channel(7,DEFAULT_ADS1x9x_ELECTRODE_INPUT,GAIN_OF_2,DEFAULT_DISABLE_POWER_DOWN,IGNORE_PREVIOUS_STATE);

  ////Initialize_ADS1x9x_Channel(8,DEFAULT_ADS1x9x_ELECTRODE_INPUT,GAIN_OF_2,DEFAULT_DISABLE_POWER_DOWN,IGNORE_PREVIOUS_STATE);

  //  if (Verify_Check == ADS_1x9x_VERIFY_ERROR)
  //  {
  //    break;                                                              // exit loop and report verify error
  //  }
  //  }
  //for (i = 0; i < ECG_Num_Channels; i++)
  //{
  //  Verify_Check = Initialize_ADS1x9x_Channel_2                               //  Context Save will store the previous setting of the channel
  //    (

  //     i + 1,                                                              // references channels 1 - 8
  //     //ADS1x9x_ONE_HALF_DIGITAL_SUPPLY,
  //      //ADS1x9x_TEST_SIGNAL ,
  //     DEFAULT_ADS1x9x_ELECTRODE_INPUT,                                    // DEFAULT_ADS1x9x_ELECTRODE_INPUT, ADS1x9x_INPUT_SHORTED, ADS1x9x_RIGHT_LEG_DETECT, ADS1x9x_ONE_HALF_DIGITAL_SUPPLY
  //     //ADS1x9x_INPUT_SHORTED,                                            // ADS1x9x_TEMPERATURE_SENSOR, ADS1x9x_CALIBRATION_SIGNAL, ADS1x9x_RIGHT_LEG_DETECT_POSITIVE, ADS1x9x_RIGHT_LEG_DETECT_NEGATIVE
  //     //DEFAULT_GAIN_OF_6,                                                          // DEFAULT_GAIN_OF_6, GAIN_OF_1, GAIN_OF_2, GAIN_OF_3, GAIN_OF_4, GAIN_OF_8, GAIN_OF_12
  //     GAIN_OF_3,
  //     DEFAULT_DISABLE_POWER_DOWN,                                         // DEFAULT_DISABLE_POWER_DOWN, ENABLE_POWER_DOWN
  //     IGNORE_PREVIOUS_STATE                                               // CONTEXT_SAVE_CHANNEL, IGNORE_PREVIOUS_STATE
  //     );
  //    if (Verify_Check == ADS_1x9x_VERIFY_ERROR)
  //      {
  //          break;                                                              // exit loop and report verify error
  //      }
  //  }


    Verify_Check = verify_ADS1x9x_Registers ((unsigned char*) ADS1x9x_download_pointer);
    Verify_Check = verify_ADS1x9x_Registers_2 ((unsigned char*) ADS1x9x_download_pointer_2);
  //disable_ADS1x9x_Interrupt ();

  return Verify_Check;
}

/**********************************************************************************************************
* Initialize ADS1298 GPIO                                                                                 *
**********************************************************************************************************/
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
    
void init_ADS1x9x_IO ()
{

  //基于单片机型号，初始化IO
	
	//本程序在mian中已初始化io完成
  //IO_Init_STM32L1XX();


  delay_ms(10);


  //__delay_cycles(102400);

  SET_ADS1298_PWDN;
  SET_ADS1298_RST;
  SET_ADS1298_CS; 
  CLR_ADS1298_START;               //输出低电平
  ADS1298_CLOCK_INTER;
  
  CLR_ADS1298_PWDN_2;
  
  ADS1298_CLOCK_EXTER_2;
  delay_ms(10);
  //__delay_cycles(102400);


  //其他的IO设置，SPI
//P3SEL |=AD_SCLK+AD_DIN+AD_DOUT;                   //端口第二功能
//P3DIR |=AD_SCLK+AD_DIN;							//输出
//P3DIR &=~AD_DOUT;   							//输入

//U0CTL |=CHAR+SYNC+MM+SWRST;						//SPI+主模式+复位
//U0TCTL |=CKPL+SSEL1+STC;							//低电平有效+SMCLK(8MHZ)+三线
//U0BR0=0X04;										//4分频
//U0BR1=0X00;										//不分频
//ME1=USPIE0;										//模块使能
//U0CTL&=~SWRST;									//开启
delay_ms(5);



}


void Power_Up_ADS1x9x ()
{
  //.....
}

void  POR_Reset_ADS1x9x ()
{
 // unsigned char t, w;
 // unsigned int i;

  //LD TODO: Use universal clock settings to set delays
  //for (t = 0x20; t > 0; t--);                                                 // Small Delay
  delay_ms(1);
  SET_ADS1298_RST;  // Reset HIGH
  SET_ADS1298_RST_2;      //AAAAAAAAAAAAAAAAAAAAAAAA
  delay_ms(10);
  //  for (i = 0xFF; i > 0; i--)
//  {
//   for (w = 0x09; w > 0; w--);                                             // Large Delay
//  }
  CLR_ADS1298_RST;                                        // Reset LOW
  CLR_ADS1298_RST_2;    //AAAAAAAAAAAAAAAAAAAAAAAAAA
  delay_ms(1);
  //for (t = 0x10; t > 0; t--);                                                 // Small Delay
  SET_ADS1298_RST;                                       // Reset HIGH
  SET_ADS1298_RST_2;      //AAAAAAAAAAAAAAAAAAAAAAAA
  delay_ms(1);
  //for (t = 0x90; t > 0; t--);

}

void Stop_Read_Data_Continuous ()
{
  Set_ADS1x9x_Chip_Enable ();
	Set_ADS1x9x_Chip_Enable_2 ();
  //S1x9x_SPI_Data (STOP_READ_DATA_CONTINUOUSLY);
  ADS1x9x_SPI_Data_All (STOP_READ_DATA_CONTINUOUSLY);
}

/**********************************************************************************************************
*                           芯片使能开关                                                                  *
**********************************************************************************************************/
void Set_ADS1x9x_Chip_Enable ()                        // ADS1x9x module uses GPIO as the SPI CS
{
  unsigned char i;
	
  CLR_ADS1298_CS;
  for (i = 5; i > 0; i--);                                                   // Short Delay before invoking the SPI Port
}

void Set_ADS1x9x_Chip_Enable_2 ()                        // ADS1x9x module uses GPIO as the SPI CS
{
  unsigned char i;
	 
  CLR_ADS1298_CS_2;
  for (i = 5; i > 0; i--);                                                  // Short Delay before invoking the SPI Port
}

void Clear_ADS1x9x_Chip_Enable ( )                      // ADS1x9x uses GPIO for SPI CS
{
  uchar i;
	
  SET_ADS1298_CS;
  for (i = 5; i > 0; i--); 
}

void Clear_ADS1x9x_Chip_Enable_2 ( )                      // ADS1x9x uses GPIO for SPI CS
{
  uchar i;
	
  SET_ADS1298_CS_2;
  for (i = 5; i > 0; i--); 
}
/**********************************************************************************************************
* *******               SPI数据发送            **********
**********************************************************************************************************/
unsigned char ADS1x9x_SPI_Data_All ( unsigned char Data)  // Complements the SPI_Address command
{
   uchar buff;
   Set_ADS1x9x_Chip_Enable();
   Set_ADS1x9x_Chip_Enable_2();
   buff = spi_readChar(Data);
   //Clear_ADS1x9x_Chip_Enable ();   // Clear the Chip ENABLE to terminate the SPI transaction
   //SET_ADS1298_CS;                //将芯片关闭，此处使用宏定义替代上面的语句，可以提高工作模式三下的采样速度
   //SET_ADS1298_CS_2;              //后添的
   
   return buff;
}

unsigned char ADS1x9x_SPI_Data ( unsigned char Data)  // Complements the SPI_Address command
{
   volatile uchar buff;
   Set_ADS1x9x_Chip_Enable();
   Clear_ADS1x9x_Chip_Enable_2();
   buff = spi_readChar(Data);
   delay_n(50);
   //Clear_ADS1x9x_Chip_Enable ();   // Clear the Chip ENABLE to terminate the SPI transaction
   SET_ADS1298_CS;                //将芯片关闭，此处使用宏定义替代上面的语句，可以提高工作模式三下的采样速度
   SET_ADS1298_CS_2;              //后添的
    
   return buff;
}

unsigned char ADS1x9x_SPI_Data_2 ( unsigned char Data)  // Complements the SPI_Address command
{

   uchar buff;
   Clear_ADS1x9x_Chip_Enable();
   Set_ADS1x9x_Chip_Enable_2();
   buff = spi_readChar(Data);
   delay_n(50);
   //Clear_ADS1x9x_Chip_Enable ();   // Clear the Chip ENABLE to terminate the SPI transaction
   SET_ADS1298_CS;                //将芯片关闭,此处使用宏定义替代上面的语句，可以提高工作模式三下的采样速度
   SET_ADS1298_CS_2;             //后添的
   
   return buff;

}

unsigned char ADS1x9x_SPI_Burst (unsigned char Data) // Complements the SPI_Address command
{                                                                               //  But allows multiple transactions (no Clear Chip ENABLE)
   uchar buff; 
   buff = spi_readChar(Data);
   delay_n(5);
   //Clear_ADS1x9x_Chip_Enable ();   // Clear the Chip ENABLE to terminate the SPI transaction
   //SET_ADS1298_CS;                //将芯片关闭,此处使用宏定义替代上面的语句，可以提高工作模式三下的采样速度
   //SET_ADS1298_CS_2;             //后添的
   
    return buff;
   

}

/**********************************************************************************************************
* ADS1298 Device SPI Interchip Communication                                                              *
*       This code is used to communicate to the ADS1298                                                   *
**********************************************************************************************************/
void ADS1x9x_SPI_Address_Byte_Count ( unsigned char Read_Write_Address, unsigned char Number_Of_Bytes)
{
  Set_ADS1x9x_Chip_Enable();
  Clear_ADS1x9x_Chip_Enable_2();
  spi_readChar(Read_Write_Address);     // Set the Chip ENABLE to start the SPI transaction 
  delay_n(10);
  spi_readChar(Number_Of_Bytes);
  delay_n(10);
}

void ADS1x9x_SPI_Address_Byte_Count_2 ( unsigned char Read_Write_Address, unsigned char Number_Of_Bytes)
{
  Clear_ADS1x9x_Chip_Enable();
  Set_ADS1x9x_Chip_Enable_2();
  spi_readChar(Read_Write_Address);     // Set the Chip ENABLE to start the SPI transaction 
  delay_n(50);
  spi_readChar(Number_Of_Bytes);
  delay_n(50);
	
}

/**********************************************************************************************************
* Initialize the ADS1x9x using a Constants Table of known good settings                        *
**********************************************************************************************************/
void init_ADS1x9x_Via_Constant_Table (unsigned char* constant_pointer)
{
  unsigned char i, j;

  ADS1x9x_SPI_Address_Byte_Count (DEFAULT_WRITE_NUMBER_OF_REGISTERS, ADS1x9x_TOP_REGISTER_SIZE);
	

  for (i = 0; i <= ADS1x9x_SPI_WRITE_DELAY; i++);                             // Delay added between SPI Writes

  for (i = 0; i < ADS1x9x_TOP_REGISTER_SIZE; i++)                            // Loop through registers to load the hex data value pairs
  {
    ADS1x9x_SPI_data = ADS1x9x_SPI_Burst (constant_pointer[i]);
    for (j = 0; j <= ADS1x9x_SPI_WRITE_DELAY; j++);                         // Delay added between SPI Writes
  }

  Clear_ADS1x9x_Chip_Enable ();                                    // Clear the Chip ENABLE to terminate any previous SPI transactions

  ADS1x9x_SPI_Address_Byte_Count ((DEFAULT_WRITE_NUMBER_OF_REGISTERS + ADS1x9x_TOP_REGISTER_SIZE + 2), ADS1x9x_BOTTOM_REGISTER_SIZE);

  for (i = 0; i <= ADS1x9x_SPI_WRITE_DELAY; i++);                             // Delay added between SPI Writes

  for (i = 0; i < ADS1x9x_BOTTOM_REGISTER_SIZE; i++)                          // Loop through registers to load the hex data value pairs
  {
    ADS1x9x_SPI_data = ADS1x9x_SPI_Burst(constant_pointer[ADS1x9x_TOP_REGISTER_SIZE+i]);
    for (j = 0; j <= ADS1x9x_SPI_WRITE_DELAY; j++);                         // Delay added between SPI Writes
  }

  Clear_ADS1x9x_Chip_Enable ();                                    // Clear the Chip ENABLE to terminate any previous SPI transactions
}


void init_ADS1x9x_Via_Constant_Table_2 (unsigned char* constant_pointer)
{
  unsigned char i, j;

  ADS1x9x_SPI_Address_Byte_Count_2 (DEFAULT_WRITE_NUMBER_OF_REGISTERS, ADS1x9x_TOP_REGISTER_SIZE);

  for (i = 0; i <= ADS1x9x_SPI_WRITE_DELAY; i++);                             // Delay added between SPI Writes

  for (i = 0; i <= ADS1x9x_TOP_REGISTER_SIZE; i++)                            // Loop through registers to load the hex data value pairs
  {
    ADS1x9x_SPI_data = ADS1x9x_SPI_Burst (constant_pointer[i]);
    for (j = 0; j <= ADS1x9x_SPI_WRITE_DELAY; j++);                         // Delay added between SPI Writes
  }

  Clear_ADS1x9x_Chip_Enable_2 ();                                    // Clear the Chip ENABLE to terminate any previous SPI transactions

  ADS1x9x_SPI_Address_Byte_Count_2 ((DEFAULT_WRITE_NUMBER_OF_REGISTERS + ADS1x9x_TOP_REGISTER_SIZE + 2), ADS1x9x_BOTTOM_REGISTER_SIZE);

  for (i = 0; i <= ADS1x9x_SPI_WRITE_DELAY; i++);                             // Delay added between SPI Writes

  for (i = 0; i < ADS1x9x_BOTTOM_REGISTER_SIZE; i++)                          // Loop through registers to load the hex data value pairs
  {
    ADS1x9x_SPI_data = ADS1x9x_SPI_Burst (constant_pointer[0x11+i]);
    for (j = 0; j <= ADS1x9x_SPI_WRITE_DELAY; j++);                         // Delay added between SPI Writes
  }

  Clear_ADS1x9x_Chip_Enable_2 ();                                    // Clear the Chip ENABLE to terminate any previous SPI transactions
}

/**********************************************************************************************************
* Verify the ADS1x9x using a Constants Table of known good settings                        *
**********************************************************************************************************/
unsigned char verify_ADS1x9x_Registers (unsigned char* constant_pointer)
{
  unsigned char i;
  unsigned char error = ADS_1x9x_INIT_SUCCESS;                                // Set Error as defaulted clear

  ADS1x9x_SPI_Address_Byte_Count ( DEFAULT_READ_NUMBER_OF_REGISTERS, ADS1x9x_TOP_REGISTER_SIZE);

  for (i = 0; i < ADS1x9x_TOP_REGISTER_SIZE; i++)                            // Loop through registers to load the hex data value pairs
  {
    ADS1x9x_SPI_data = ADS1x9x_SPI_Burst ( SPI_TEST_DATA);
    ADS1x9x_Default_Register_Watch[i] = ADS1x9x_SPI_data;

    if (ADS1x9x_SPI_data != constant_pointer [i])                           // Do the values match?  Assuming the Addresses match
    {
      error = ADS_1x9x_VERIFY_ERROR;                                      // Set the Error
    }
  }
  Clear_ADS1x9x_Chip_Enable ();                                    // Clear the Chip ENABLE to terminate any previous SPI transactions

  ADS1x9x_SPI_Address_Byte_Count ( (DEFAULT_READ_NUMBER_OF_REGISTERS + ADS1x9x_TOP_REGISTER_SIZE + 2), ADS1x9x_BOTTOM_REGISTER_SIZE);

  for (i = 0; i < ADS1x9x_BOTTOM_REGISTER_SIZE; i++)                          // Loop through registers to load the hex data value pairs
  {
    ADS1x9x_SPI_data = ADS1x9x_SPI_Burst ( SPI_TEST_DATA);
    ADS1x9x_Default_Register_Watch[i+0x11] = ADS1x9x_SPI_data;
    if (ADS1x9x_SPI_data != constant_pointer [0x11+i])     // Do the values match?  Assuming the Addresses match
    {
      error = ADS_1x9x_VERIFY_ERROR;                                      // Set the Error
    }
  }
  Clear_ADS1x9x_Chip_Enable ();                                    // Clear the Chip ENABLE to terminate any previous SPI transactions

  return error;                                                               // Return the error back from the routine
}

unsigned char verify_ADS1x9x_Registers_2 (unsigned char* constant_pointer)
{
  unsigned char i;
  unsigned char error = ADS_1x9x_INIT_SUCCESS;                                // Set Error as defaulted clear

  ADS1x9x_SPI_Address_Byte_Count_2 ( DEFAULT_READ_NUMBER_OF_REGISTERS, ADS1x9x_TOP_REGISTER_SIZE);

  for (i = 0; i <= ADS1x9x_TOP_REGISTER_SIZE; i++)                            // Loop through registers to load the hex data value pairs
  {
    ADS1x9x_SPI_data = ADS1x9x_SPI_Burst ( SPI_TEST_DATA);
		ADS1x9x_Default_Register_Watch2[i] = ADS1x9x_SPI_data;
    if (ADS1x9x_SPI_data != constant_pointer [i])                           // Do the values match?  Assuming the Addresses match
    {
      error = ADS_1x9x_VERIFY_ERROR;                                      // Set the Error
    }
  }
  Clear_ADS1x9x_Chip_Enable_2 ();                                    // Clear the Chip ENABLE to terminate any previous SPI transactions

  ADS1x9x_SPI_Address_Byte_Count_2 ( (DEFAULT_READ_NUMBER_OF_REGISTERS + ADS1x9x_TOP_REGISTER_SIZE + 2), ADS1x9x_BOTTOM_REGISTER_SIZE);

  for (i = 0; i < ADS1x9x_BOTTOM_REGISTER_SIZE; i++)                          // Loop through registers to load the hex data value pairs
  {
    ADS1x9x_SPI_data = ADS1x9x_SPI_Burst ( SPI_TEST_DATA);
		ADS1x9x_Default_Register_Watch2[i+0x11] = ADS1x9x_SPI_data;
    if (ADS1x9x_SPI_data != constant_pointer [i+0x11])     // Do the values match?  Assuming the Addresses match
    {
      error = ADS_1x9x_VERIFY_ERROR;                                      // Set the Error
    }
  }
  Clear_ADS1x9x_Chip_Enable_2 ();                                    // Clear the Chip ENABLE to terminate any previous SPI transactions

  return error;                                                               // Return the error back from the routine
}
/**********************************************************************************************************
*               ADS1x9x Data Rate                                                                         *
**********************************************************************************************************/

unsigned char Initialize_ADS1x9x_Data_Rate (unsigned char Modulation_Frequency_Divider)
{
  unsigned char Verify_status = ADS_1x9x_INIT_SUCCESS;                                    // Error state set Clear

  // Load Values set in the ADS1x9x
  ADS1x9x_SPI_Address_Byte_Count (READ_CONFIG_1_REGISTER, SINGLE_BYTE_READ_WRITE);        // Read Device ID, Single Byte the Part Number
  ADS1x9x_Config_1 = ADS1x9x_SPI_Data (SPI_TEST_DATA);                                    // Send Dummy variable (0x55) to return the part number (Chip Select Cleared automatically)

  //  Modify Values for the ADS1x9x
  ADS1x9x_Config_1_register->Output_Data_Rate = Modulation_Frequency_Divider;             // DEFAULT_MODULATION_FREQUENCY_DIVIDED_BY_16
                                                                                          // MODULATION_FREQUENCY_DIVIDED_BY_32
                                                                                          // MODULATION_FREQUENCY_DIVIDED_BY_64
                                                                                          // MODULATION_FREQUENCY_DIVIDED_BY_128
                                                                                          // MODULATION_FREQUENCY_DIVIDED_BY_256
                                                                                          // MODULATION_FREQUENCY_DIVIDED_BY_512
                                                                                          // MODULATION_FREQUENCY_DIVIDED_BY_1024

  // Program Values into ADS1x9x
  ADS1x9x_SPI_Address_Byte_Count (WRITE_CONFIG_1_REGISTER, SINGLE_BYTE_READ_WRITE);      // Read Device ID, Single Byte the Part Number
  ADS1x9x_SPI_data = ADS1x9x_SPI_Data (ADS1x9x_Config_1);                                // Send Dummy variable (0x55) to return the part number (Chip Select Cleared automatically)


#ifdef VERIFY
  //  Read Back Register
  ADS1x9x_SPI_Address_Byte_Count (READ_CONFIG_1_REGISTER, SINGLE_BYTE_READ_WRITE);       // Read Device ID, Single Byte the Part Number
  ADS1x9x_SPI_data = ADS1x9x_SPI_Data (SPI_TEST_DATA);                                   // Read the Value from the SPI port
  if (ADS1x9x_SPI_data != ADS1x9x_Config_1)
  {
    Verify_status = ADS_1x9x_VERIFY_ERROR;
  }
  //  -----------------------------------
#endif /* VERIFY */
  return Verify_status;
}



unsigned char Initialize_ADS1x9x_Data_Rate_2 (unsigned char Modulation_Frequency_Divider)
{
  unsigned char Verify_status = ADS_1x9x_INIT_SUCCESS;                                    // Error state set Clear

  // Load Values set in the ADS1x9x
  ADS1x9x_SPI_Address_Byte_Count_2 (READ_CONFIG_1_REGISTER, SINGLE_BYTE_READ_WRITE);        // Read Device ID, Single Byte the Part Number
  ADS1x9x_Config_1 = ADS1x9x_SPI_Data (SPI_TEST_DATA);                                    // Send Dummy variable (0x55) to return the part number (Chip Select Cleared automatically)

  //  Modify Values for the ADS1x9x
  ADS1x9x_Config_1_register->Output_Data_Rate = Modulation_Frequency_Divider;             // DEFAULT_MODULATION_FREQUENCY_DIVIDED_BY_16
                                                                                          // MODULATION_FREQUENCY_DIVIDED_BY_32
                                                                                          // MODULATION_FREQUENCY_DIVIDED_BY_64
                                                                                          // MODULATION_FREQUENCY_DIVIDED_BY_128
                                                                                          // MODULATION_FREQUENCY_DIVIDED_BY_256
                                                                                          // MODULATION_FREQUENCY_DIVIDED_BY_512
                                                                                          // MODULATION_FREQUENCY_DIVIDED_BY_1024

  // Program Values into ADS1x9x
  ADS1x9x_SPI_Address_Byte_Count_2 (WRITE_CONFIG_1_REGISTER, SINGLE_BYTE_READ_WRITE);      // Read Device ID, Single Byte the Part Number
  ADS1x9x_SPI_data = ADS1x9x_SPI_Data_2 (ADS1x9x_Config_1);                                // Send Dummy variable (0x55) to return the part number (Chip Select Cleared automatically)


#ifdef VERIFY
  //  Read Back Register
  ADS1x9x_SPI_Address_Byte_Count_2 (READ_CONFIG_1_REGISTER, SINGLE_BYTE_READ_WRITE);       // Read Device ID, Single Byte the Part Number
  ADS1x9x_SPI_data = ADS1x9x_SPI_Data_2 (SPI_TEST_DATA);                                   // Read the Value from the SPI port
  if (ADS1x9x_SPI_data != ADS1x9x_Config_1)
  {
    Verify_status = ADS_1x9x_VERIFY_ERROR;
  }
  //  -----------------------------------
#endif /* VERIFY */
  return Verify_status;
}

/**********************************************************************************************************
*               ADS1x9x Status Registers                                                                  *
**********************************************************************************************************/
unsigned char ADS1x9x_Read_Version ()
{
  unsigned char Version_ID = 0;
  ADS1x9x_SPI_Address_Byte_Count ( 0X20, 0X00);    // Read Device ID, Single Byte the Part Number
  Version_ID = ADS1x9x_SPI_Data ( SPI_TEST_DATA);                              // Send Dummy variable (0x55) to return the part number (Chip Select Cleared automatically)
  return Version_ID;
}

unsigned char ADS1x9x_Read_Version_2 ()
{
  unsigned char Version_ID = 0;
   Set_ADS1x9x_Chip_Enable_2 ();                                              // Set the Chip ENABLE to start the SPI transaction
  ADS1x9x_SPI_Address_Byte_Count_2(0x20,0x00);
  Version_ID = ADS1x9x_SPI_Data_2 ( SPI_TEST_DATA); 
  
                              // Send Dummy variable (0x55) to return the part number (Chip Select Cleared automatically)
  return Version_ID;
}
/**********************************************************************************************************
*                ADS1x9x High_Performance LOW_POWER Mode                                                  *
**********************************************************************************************************/
unsigned char Initialize_ADS1x9x_Mode ( unsigned char ADC_Power_Mode)
{
  unsigned char Verify_status = ADS_1x9x_INIT_SUCCESS;                                                // Error state set Clear

  // Load Values set in the ADS1x9x
  ADS1x9x_SPI_Address_Byte_Count (READ_CONFIG_1_REGISTER, SINGLE_BYTE_READ_WRITE);       // Read Device ID, Single Byte the Part Number
  ADS1x9x_Config_1 = ADS1x9x_SPI_Data (SPI_TEST_DATA);                                   // Send Dummy variable (0x55) to return the part number (Chip Select Cleared automatically)

  //  Modify Values for the ADS1x9x
  ADS1x9x_Config_1_register->Power_Resolution_Optimization = ADC_Power_Mode;                          // Default_Low_Power_Mode, High_Resolution_Mode

  // Program Values into ADS1x9x
  ADS1x9x_SPI_Address_Byte_Count (WRITE_CONFIG_1_REGISTER, SINGLE_BYTE_READ_WRITE);      // Read Device ID, Single Byte the Part Number
  ADS1x9x_SPI_data = ADS1x9x_SPI_Data (ADS1x9x_Config_1);                                // Send Dummy variable (0x55) to return the part number (Chip Select Cleared automatically)


#ifdef VERIFY
  //  Read Back Register
  ADS1x9x_SPI_Address_Byte_Count (READ_CONFIG_1_REGISTER, SINGLE_BYTE_READ_WRITE);        // Read Device ID, Single Byte the Part Number
  ADS1x9x_SPI_data = ADS1x9x_SPI_Data (SPI_TEST_DATA);                                    // Read the Value from the SPI port
  if (ADS1x9x_SPI_data != ADS1x9x_Config_1)
  {
    Verify_status = ADS_1x9x_VERIFY_ERROR;
  }
  //  -----------------------------------
#endif /* VERIFY */
  return Verify_status;
}


unsigned char Initialize_ADS1x9x_Mode_2 ( unsigned char ADC_Power_Mode)
{
  unsigned char Verify_status = ADS_1x9x_INIT_SUCCESS;                                                // Error state set Clear

  // Load Values set in the ADS1x9x
  ADS1x9x_SPI_Address_Byte_Count_2 (READ_CONFIG_1_REGISTER, SINGLE_BYTE_READ_WRITE);       // Read Device ID, Single Byte the Part Number
  ADS1x9x_Config_1 = ADS1x9x_SPI_Data (SPI_TEST_DATA);                                   // Send Dummy variable (0x55) to return the part number (Chip Select Cleared automatically)

  //  Modify Values for the ADS1x9x
  ADS1x9x_Config_1_register->Power_Resolution_Optimization = ADC_Power_Mode;                          // Default_Low_Power_Mode, High_Resolution_Mode

  // Program Values into ADS1x9x
  ADS1x9x_SPI_Address_Byte_Count_2 (WRITE_CONFIG_1_REGISTER, SINGLE_BYTE_READ_WRITE);      // Read Device ID, Single Byte the Part Number
  ADS1x9x_SPI_data = ADS1x9x_SPI_Data (ADS1x9x_Config_1);                                // Send Dummy variable (0x55) to return the part number (Chip Select Cleared automatically)


#ifdef VERIFY
  //  Read Back Register
  ADS1x9x_SPI_Address_Byte_Count_2 (READ_CONFIG_1_REGISTER, SINGLE_BYTE_READ_WRITE);        // Read Device ID, Single Byte the Part Number
  ADS1x9x_SPI_data = ADS1x9x_SPI_Data (SPI_TEST_DATA);                                    // Read the Value from the SPI port
  if (ADS1x9x_SPI_data != ADS1x9x_Config_1)
  {
    Verify_status = ADS_1x9x_VERIFY_ERROR;
  }
  //  -----------------------------------
#endif /* VERIFY */
  return Verify_status;
}

/**********************************************************************************************************
*                ADS1x9x Channel Initialization                                                           *
**********************************************************************************************************/
unsigned char Initialize_ADS1x9x_Channel
(
unsigned char Channel_Number,
unsigned char Input_Type,
unsigned char Input_Gain,
unsigned char Enable_Power_Setting,
unsigned char Save_Previous_Setting
)
{
  unsigned char SPI_Read_Register = 0x00;
  unsigned char SPI_Write_Register = 0x00;
  unsigned char Verify_status = ADS_1x9x_INIT_SUCCESS;                        // Error state set Clear

  // Set SPI Address to Read and Write
  switch (Channel_Number)
  {
  case 1:
    SPI_Read_Register = READ_CHANNEL_1_SET_REGISTER;
    SPI_Write_Register = WRITE_CHANNEL_1_SET_REGISTER;
    break;
  case 2:
    SPI_Read_Register = READ_CHANNEL_2_SET_REGISTER;
    SPI_Write_Register = WRITE_CHANNEL_2_SET_REGISTER;
    break;
  case 3:
    SPI_Read_Register = READ_CHANNEL_3_SET_REGISTER;
    SPI_Write_Register = WRITE_CHANNEL_3_SET_REGISTER;
    break;    	
  case 4:
    SPI_Read_Register = READ_CHANNEL_4_SET_REGISTER;
    SPI_Write_Register = WRITE_CHANNEL_4_SET_REGISTER;
    break;
  case 5:
    SPI_Read_Register = READ_CHANNEL_5_SET_REGISTER;
    SPI_Write_Register = WRITE_CHANNEL_5_SET_REGISTER;
    break;
  case 6:
    SPI_Read_Register = READ_CHANNEL_6_SET_REGISTER;
    SPI_Write_Register = WRITE_CHANNEL_6_SET_REGISTER;
    break;
  case 7:
    SPI_Read_Register = READ_CHANNEL_7_SET_REGISTER;
    SPI_Write_Register = WRITE_CHANNEL_7_SET_REGISTER;
    break;
  case 8:
    SPI_Read_Register = READ_CHANNEL_8_SET_REGISTER;
    SPI_Write_Register = WRITE_CHANNEL_8_SET_REGISTER;
    break;
  default:
    break;
  }

  // Load Values set in the ADS1x9x
  ADS1x9x_SPI_Address_Byte_Count (SPI_Read_Register, SINGLE_BYTE_READ_WRITE);    // Read Device ID, Single Byte the Part Number
  ADS1x9x_Channel_Settings = ADS1x9x_SPI_Data (SPI_TEST_DATA);                   // Send Dummy variable (0x55) to return the part number (Chip Select Cleared automatically)

  if (Save_Previous_Setting == CONTEXT_SAVE_CHANNEL)
  {
    ADS_1298_Channel_Stack [Channel_Number - 1] = ADS1x9x_Channel_Settings;             // Context Save Channel Setting
  }

  //  Modify Values for the ADS1x9x
  ADS1x9x_Channel_Settings_Register->Channel_Input_Is = Input_Type;                       // DEFAULT_ADS1x9x_ELECTRODE_INPUT, ADS1x9x_INPUT_SHORTED
  // ADS1x9x_RIGHT_LEG_DETECT, ADS1x9x_ONE_HALF_DIGITAL_SUPPLY
  // ADS1x9x_TEMPERATURE_SENSOR, ADS1x9x_CALIBRATION_SIGNAL
  // ADS1x9x_RIGHT_LEG_DETECT_POSITIVE, ADS1x9x_RIGHT_LEG_DETECT_NEGATIVE
  ADS1x9x_Channel_Settings_Register->Programmable_Gain_Setting = Input_Gain;              // DEFAULT_GAIN_OF_6, GAIN_OF_1, GAIN_OF_2, GAIN_OF_3,
  // GAIN_OF_4, GAIN_OF_8, GAIN_OF_12
  ADS1x9x_Channel_Settings_Register->Power_Down_Channel = Enable_Power_Setting;           // DEFAULT_DISABLE_POWER_DOWN, ENABLE_POWER_DOWN, ENABLE

  // Program Values into ADS1x9x
  ADS1x9x_SPI_Address_Byte_Count (SPI_Write_Register, SINGLE_BYTE_READ_WRITE);   // Read Device ID, Single Byte the Part Number
  ADS1x9x_SPI_data = ADS1x9x_SPI_Data (ADS1x9x_Channel_Settings);                // Send Dummy variable (0x55) to return the part number (Chip Select Cleared automatically)


#ifdef VERIFY
  //  Read Back Register
  ADS1x9x_SPI_Address_Byte_Count (SPI_Read_Register, SINGLE_BYTE_READ_WRITE);    // Read Device ID, Single Byte the Part Number
  ADS1x9x_SPI_data = ADS1x9x_SPI_Data (SPI_TEST_DATA);                           // Read the Value from the SPI port
  if (ADS1x9x_SPI_data != ADS1x9x_Channel_Settings)
  {
    Verify_status = ADS_1x9x_VERIFY_ERROR;
  }
  //  -----------------------------------
#endif /* VERIFY */
  return Verify_status;
}


unsigned char Initialize_ADS1x9x_Channel_2
(
unsigned char Channel_Number,
unsigned char Input_Type,
unsigned char Input_Gain,
unsigned char Enable_Power_Setting,
unsigned char Save_Previous_Setting
)
{
  unsigned char SPI_Read_Register = 0x00;
  unsigned char SPI_Write_Register = 0x00;
  unsigned char Verify_status = ADS_1x9x_INIT_SUCCESS;                        // Error state set Clear

  // Set SPI Address to Read and Write
  switch (Channel_Number)
  {
  case 1:
    SPI_Read_Register = READ_CHANNEL_1_SET_REGISTER;
    SPI_Write_Register = WRITE_CHANNEL_1_SET_REGISTER;
    break;
  case 2:
    SPI_Read_Register = READ_CHANNEL_2_SET_REGISTER;
    SPI_Write_Register = WRITE_CHANNEL_2_SET_REGISTER;
    break;
  case 3:
    SPI_Read_Register = READ_CHANNEL_3_SET_REGISTER;
    SPI_Write_Register = WRITE_CHANNEL_3_SET_REGISTER;
    break;    	
  case 4:
    SPI_Read_Register = READ_CHANNEL_4_SET_REGISTER;
    SPI_Write_Register = WRITE_CHANNEL_4_SET_REGISTER;
    break;
  case 5:
    SPI_Read_Register = READ_CHANNEL_5_SET_REGISTER;
    SPI_Write_Register = WRITE_CHANNEL_5_SET_REGISTER;
    break;
  case 6:
    SPI_Read_Register = READ_CHANNEL_6_SET_REGISTER;
    SPI_Write_Register = WRITE_CHANNEL_6_SET_REGISTER;
    break;
  case 7:
    SPI_Read_Register = READ_CHANNEL_7_SET_REGISTER;
    SPI_Write_Register = WRITE_CHANNEL_7_SET_REGISTER;
    break;
  case 8:
    SPI_Read_Register = READ_CHANNEL_8_SET_REGISTER;
    SPI_Write_Register = WRITE_CHANNEL_8_SET_REGISTER;
    break;
  default:
    break;
  }

  // Load Values set in the ADS1x9x
  ADS1x9x_SPI_Address_Byte_Count_2 (SPI_Read_Register, SINGLE_BYTE_READ_WRITE);    // Read Device ID, Single Byte the Part Number
  ADS1x9x_Channel_Settings = ADS1x9x_SPI_Data_2 (SPI_TEST_DATA);                   // Send Dummy variable (0x55) to return the part number (Chip Select Cleared automatically)

  if (Save_Previous_Setting == CONTEXT_SAVE_CHANNEL)
  {
    ADS_1298_Channel_Stack [Channel_Number - 1] = ADS1x9x_Channel_Settings;             // Context Save Channel Setting
  }

  //  Modify Values for the ADS1x9x
  ADS1x9x_Channel_Settings_Register->Channel_Input_Is = Input_Type;                       // DEFAULT_ADS1x9x_ELECTRODE_INPUT, ADS1x9x_INPUT_SHORTED
  // ADS1x9x_RIGHT_LEG_DETECT, ADS1x9x_ONE_HALF_DIGITAL_SUPPLY
  // ADS1x9x_TEMPERATURE_SENSOR, ADS1x9x_CALIBRATION_SIGNAL
  // ADS1x9x_RIGHT_LEG_DETECT_POSITIVE, ADS1x9x_RIGHT_LEG_DETECT_NEGATIVE
  ADS1x9x_Channel_Settings_Register->Programmable_Gain_Setting = Input_Gain;              // DEFAULT_GAIN_OF_6, GAIN_OF_1, GAIN_OF_2, GAIN_OF_3,
  // GAIN_OF_4, GAIN_OF_8, GAIN_OF_12
  ADS1x9x_Channel_Settings_Register->Power_Down_Channel = Enable_Power_Setting;           // DEFAULT_DISABLE_POWER_DOWN, ENABLE_POWER_DOWN, ENABLE

  // Program Values into ADS1x9x
  ADS1x9x_SPI_Address_Byte_Count_2 (SPI_Write_Register, SINGLE_BYTE_READ_WRITE);   // Read Device ID, Single Byte the Part Number
  ADS1x9x_SPI_data = ADS1x9x_SPI_Data_2 (ADS1x9x_Channel_Settings);                // Send Dummy variable (0x55) to return the part number (Chip Select Cleared automatically)


#ifdef VERIFY
  //  Read Back Register
  ADS1x9x_SPI_Address_Byte_Count_2 (SPI_Read_Register, SINGLE_BYTE_READ_WRITE);    // Read Device ID, Single Byte the Part Number
  ADS1x9x_SPI_data = ADS1x9x_SPI_Data_2 (SPI_TEST_DATA);                           // Read the Value from the SPI port
  if (ADS1x9x_SPI_data != ADS1x9x_Channel_Settings)
  {
    Verify_status = ADS_1x9x_VERIFY_ERROR;
  }
  //  -----------------------------------
#endif /* VERIFY */
  return Verify_status;
}

void set_ADS1x9x_Start_Pin (unsigned char state)
{
  if(state==HIGH)
    SET_ADS1298_START;
  else
    CLR_ADS1298_START;                           // Start Pin LOW
}

