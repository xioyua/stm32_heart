#include "include.h"
//该文件使用普通UART
//因为使用DMAUART，所以这个文件失去了作用
static uchar BG[2] ={'B','G'};
uchar RSerChar = 0;



	
//***************************************************************************//
//**name    :void Send_ChannelData(char Channel)
//**function  :
//**describe  :
//**return  :
//***************************************************************************//
void Send_ChannelData(uchar channel)
{
	uchar byte1,byte2,byte3;
	uchar outByteH,outByteL;
    uint sum;
		if(channel!= 0 && channel != 9 && channel!= 2 && channel != 3 && channel != 10 && channel != 11)
		{
			byte1 = ADS1x9x_Data[channel*3];
			byte2 = ADS1x9x_Data[channel*3+1];
			byte3 = ADS1x9x_Data[channel*3+2];
            if((byte1 & 0x80) == 0x80)
            {
                sum = ((byte1 & 0x7fffff) <<16 ) + (byte2<<8) + byte3;
            }
            else
            {
                sum = (byte1 <<16 ) + (byte2<<8) + byte3;
                sum = sum + 0x800000;
            }
			//需要12BIT,取首字节的最后2位，二字节全部八位，尾字节前2位
			//byte1 = byte1 & 0x03;
			//byte3 = byte3 & 0xc0;
			//outByteH = (byte1<<2) + ((byte2 & 0xc0 )>>6);
			//outByteL = (byte2<<2) + (byte3>>6);
 			outByteH = (sum & 0x00ff00)>>8;
 			outByteL = sum & 0x0000ff;
			HAL_UART_Transmit(&huart1,&outByteH,sizeof(channel),50);
			HAL_UART_Transmit(&huart1,&outByteL,sizeof(channel),50);
		}
		else if (channel == 2 || channel == 3)
		{
			if((byte1 & 0x80) == 0x80)
            {
                sum = ((byte1 & 0x7fffff) <<16 ) + (byte2<<8) + byte3;
            }
            else
            {
                sum = (byte1 <<16 ) + (byte2<<8) + byte3;
                sum = sum + 0x800000;
            }
			//需要12BIT,取首字节的最后2位，二字节全部八位，尾字节前2位
			//byte1 = byte1 & 0x03;
			//byte3 = byte3 & 0xc0;
			//outByteH = (byte1<<2) + ((byte2 & 0xc0 )>>6);
			//outByteL = (byte2<<2) + (byte3>>6);
 			outByteH = (sum & 0x00ff00)>>8;
 			outByteL = sum & 0x0000ff;
			HAL_UART_Transmit(&huart1,&outByteH,sizeof(channel),50);
			HAL_UART_Transmit(&huart1,&outByteL,sizeof(channel),50);
		}
		
}

//***************************************************************************//
//**name    :void Send_Data_ALL()
//**function  :
//**describe  :
//**return  :
//***************************************************************************//
void Send_Data_ALL()
{
	HAL_UART_Transmit(&huart1,&BG[0],sizeof(BG[0]),50);
	HAL_UART_Transmit(&huart1,&BG[1],sizeof(BG[1]),50);
	Send_ChannelData(3);
	Send_ChannelData(2);
	Send_ChannelData(4);
	Send_ChannelData(5);
	Send_ChannelData(6);
	Send_ChannelData(7);
	Send_ChannelData(8);
	Send_ChannelData(1);
	Send_ChannelData(13);
	Send_ChannelData(14);
	Send_ChannelData(15);
	Send_ChannelData(16);
	Send_ChannelData(17);
	Send_ChannelData(12);

}





