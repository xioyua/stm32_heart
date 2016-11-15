#include "include.h"
/* Private define ------------------------------------------------------------*/
//WRITER:熊源

/* Private macro -------------------------------------------------------------*/
/* ----------------------------------------变量 ---------------------------------------------------------*/

volatile uint32_t  ADS1x9x_Data [CHANNEL_SEL];  				//ADS1298通道数据缓冲
uint8_t  ADS1x9x_Data_Smoothing[ADS1298_DATA_LENGTH]; 	//用于数据滤波的缓冲
volatile unsigned char ADS1298_lsData[ADS1298_LASTDATA_LWNGTH];//用于存放最后要发送的数据
int32_t      ADS1x9x_Data_base[CHANNEL_SEL];
unsigned char Serial_Buffer=0;                     //判断串口命令的标志
unsigned int  Timer_Flag =0;                       //时钟标志
unsigned char ADS1298_ON_OFF=1;                    //开关
unsigned int  Sample_Number=0;                      //记录转换次数
unsigned char Send_Flag=1;                         //自己控制的转换使能标志
unsigned char shuz[2];                             //在子函数Deal_Data中使用

unsigned char LSRegister;
unsigned char loffP[18];
unsigned char loffN[18];

/* ------------------------------------------函数声明 -----------------------------------------------*/


char		  Double_Chip_Data_get(uint8_t * sumBuf);      //两芯片数据采集
void 					Send_Data(char Channel);         //串口发送数据
// void 					DMA_Init();
unsigned char getRegister(unsigned char num);  //读取寄存器的值
void 					getLoffP(unsigned char *c);
void 					getLoffN(unsigned char *c);
void 					copyToHalfHighByte(unsigned char shuz[],unsigned char loffP,unsigned char loffN);
void 					lastSend(unsigned char num);      //将处理后的数据通过串口发送
/*------------------------------函数定义---------------------------------------------*/
void spi_sendChar(uchar data)
{
    HAL_SPI_Transmit(&hspi1,&data,sizeof(data),3);	 
	delay_n(10);
}

uchar spi_readChar(uchar data)
{
    uchar redata;
	HAL_SPI_TransmitReceive(&hspi1,&data,&redata,sizeof(data),3);	
	delay_n(5);
//     for(i=0;i<8;i++)
//     {
//         if(redata&0x80)
//         {
//             buf = buf + (0x01<<i);
//         }
//         redata= redata<<1;
//     }
    return redata;
}
//***************************************************************************//
//**函数名    ：void Start_Sample(void)
//**函数功能  ：发送采集命令，将采集和接受分开，中间的等待时间可以用来进行串口传输
//**入口参数  ：无
//**返回参数  ：无
//**数据存储全局变量 : ADS1x9x_Data[ADS1298_DATA_LENGTH]
//***************************************************************************//
void Start_Sample_Mannully()
{
  CLR_ADS1298_CS;
  #if CHANNEL_SEL == CHANNEL16
  CLR_ADS1298_CS_2;
  #endif
  delay_n(20);
  spi_readChar( READ_DATA_MANUALLY);//手动读数据
  SET_ADS1298_CS;
  SET_ADS1298_CS_2;
}

void Start_Sample()
{
  CLR_ADS1298_CS;
  #if CHANNEL_SEL == CHANNEL16
  CLR_ADS1298_CS_2;
  #endif
  delay_n(20);
  spi_readChar( SET_READ_DATA_CONTINUOUSLY);//连续读数据模式
  SET_ADS1298_CS;
  SET_ADS1298_CS_2;
}
//***************************************************************************//
//**函数名    ：void Double_Chip_Data_get(void)
//**函数功能  ：两块芯片联合采集数据，共54字节，有用数据48字节，16通道
//**入口参数  ：无
//**返回参数  ：无
//**数据存储全局变量 : ADS1x9x_Data[ADS1298_DATA_LENGTH]
//***************************************************************************//
char Double_Chip_Data_get(uint8_t  sumBuf[])
{
  unsigned char i = 0 ;
  
  
  
    {
        //等待
          while (ADS1298_READY_FLAG!=0 );
          #if CHANNEL_SEL == CHANNEL16
		      while (ADS1298_READY_FLAG_2!=0 );
          #endif

          CLR_ADS1298_CS;
          #if CHANNEL_SEL == CHANNEL16
          CLR_ADS1298_CS_2;
          #endif
          delay_n(20);
          spi_readChar( READ_DATA_MANUALLY);

          

          
          CLR_ADS1298_CS;
          SET_ADS1298_CS_2;
          delay_n(20);
          for (i=0;i<27;i++)
            {
                
                sumBuf[i] =spi_readChar(SPI_TEST_DATA);
                delay_n(5);
             }
         
          #if CHANNEL_SEL == CHANNEL16
					SET_ADS1298_CS;
          CLR_ADS1298_CS_2;
          
          delay_n(100);
          for (i=27;i<ADS1298_DATA_LENGTH;i++)
             {
                 sumBuf[i] =spi_readChar(SPI_TEST_DATA);
              }
          
          #endif        
          SET_ADS1298_CS;
          SET_ADS1298_CS_2;
     }
  return 1;
}  

//***************************************************************************//
//**函数名    ：void Deal_Data(char data)
//**函数功能  ：将数据右移4位，最高位置零
//**入口参数  ：数据
//**返回参数  ：无
//***************************************************************************//
#if FORMAT == BIT12
void Deal_Data()
{
   unsigned char i;
   //字头
   ADS1298_lsData[0]='B';
   ADS1298_lsData[1]='G';

   //通道2顺序第2
   ADS1298_lsData[4] = (ADS1x9x_Data[1] &0x00f000)>>12;
   ADS1298_lsData[5] = ((ADS1x9x_Data[1] & 0x000ff0 )>>4) ;
   //通道3顺序1
   ADS1298_lsData[2] = (ADS1x9x_Data[2] &0x00f000)>>12;
   ADS1298_lsData[3] = ((ADS1x9x_Data[2] & 0x000ff0 )>>4) ;
   //通道4-8
   for(i=3;i<=7;i++)
   {
        ADS1298_lsData[(i)*2] = (ADS1x9x_Data[i] &0x00f000)>>12;
        ADS1298_lsData[(i)*2+1] = ((ADS1x9x_Data[i] & 0x000ff0 )>>4) ;
   }
   //通道1
   ADS1298_lsData[16]=(ADS1x9x_Data[0] &0x00f000)>>12;
   ADS1298_lsData[17]=((ADS1x9x_Data[0] & 0x000ff0 )>>4) ;
   #if CHANNEL_SEL == CHANNEL16
   //通道12-16
    for(i=9;i<=13;i++)
    {
        ADS1298_lsData[(i)*2] = (ADS1x9x_Data[i+2] &0x00f000)>>12;
        ADS1298_lsData[(i)*2+1] =((ADS1x9x_Data[i+2] & 0x000ff0 )>>4);
    }
    //通道11
    ADS1298_lsData[28]=(ADS1x9x_Data[10] &0x00f000)>>12;
    ADS1298_lsData[29]=((ADS1x9x_Data[10] & 0x000ff0 )>>4) ;
   #endif


}
#elif FORMAT == BIT16

void Deal_Data()
{
   unsigned char i;
   
   //通道2-8
   for(i=0;i<=6;i++)
   {
        ADS1298_lsData[(i)*2+1] =  (ADS1x9x_Data[(i+1)]&0xff00)>>8;  //高位
        ADS1298_lsData[(i)*2] = (ADS1x9x_Data[i+1]&0x00ff);         //低位
        ADS1298_lsData[(i)*2+1] = (0x80 & ADS1298_lsData[(i)*2+1]) | ((0x3e & ADS1298_lsData[(i)*2+1]) <<1)|((ADS1298_lsData[(i)*2]&0x80)>>7);
        ADS1298_lsData[(i)*2] = (ADS1298_lsData[(i)*2] & 0x7f)<<1;
   }
   //通道1
   ADS1298_lsData[15] =  (ADS1x9x_Data[(0)]&0x7f00)>>8;
   ADS1298_lsData[14] = (ADS1x9x_Data[0]&0x00ff);
   ADS1298_lsData[15] = (0x80 & ADS1298_lsData[15]) | ((0x3e & ADS1298_lsData[15])<<1) |((ADS1298_lsData[14]&0x80)>>7);
   ADS1298_lsData[14] = (ADS1298_lsData[14] & 0x7f)<<1;

   #if CHANNEL_SEL == CHANNEL16
   //通道12-16
    for(i=8;i<=12;i++)
    {
        ADS1298_lsData[(i)*2+1] = ADS1x9x_Data[(i+5)*3] + 0x80;
        ADS1298_lsData[(i)*2] = ADS1x9x_Data[(i+5)*3+1];
    }
    //通道11
    ADS1298_lsData[27]=ADS1x9x_Data[36] + 0x80;
    ADS1298_lsData[26]=ADS1x9x_Data[37];
   #endif

}
#endif
//***************************************************************************//
//**函数名    ：void Sample_And_Smoothing(uint32_t* distination,uint8_t* sumBuf,uchar num)
//**函数功能  ：采集并处理
//**入口参数  ：数据
//**返回参数  ：无
//***************************************************************************//

void Sample_And_Smoothing(uint32_t* distination,uint8_t* sumBuf,uchar num)
{
    uchar i;
    //多次采集，每个通道的数据都相加于其对应的sumBuf中
    for(i=0;i<CHANNEL_SEL;i++)
	{
			distination[i] = 0;
	}
        //set_ADS1x9x_Start_Pin(LOW);               //置低STARTpin
		//Start_Sample_Mannully();
        Double_Chip_Data_get(sumBuf);
        for(i=0;i<CHANNEL_SEL;i++)
        {
            if(i<8)
            {      
                distination[i] = (((sumBuf[(i+1)*3]<<16)+(sumBuf[(i+1)*3+1]<<8) + sumBuf[(i+1)*3+2]));	
            }
            else
            {
                distination[i] = (((sumBuf[(i+2)*3]<<16)+(sumBuf[(i+2)*3+1]<<8) + sumBuf[(i+2)*3+2]));
            }
        }
       // set_ADS1x9x_Start_Pin(HIGH);               //置低STARTpin
    
    //求平均值，存放在ADS1x9x_Data里
    
		
    
}

void ADS1298_Sample(void)
{
		uchar i;
    Sample_And_Smoothing(ADS1x9x_Data,ADS1x9x_Data_Smoothing,1);
		for(i=0;i<CHANNEL_SEL;i++)
    {
        ADS1x9x_Data[i] = (ADS1x9x_Data[i] - ADS1x9x_Data_base[i]) ;	
        
    }
    Deal_Data();
}
//去基线函数：
//		根据第一次取值，得到与所设基线值(0x017777)之间的差在之后读取的数据中，将自动减去这个差值
void ADS1298_DetectBase(void)
{
    uchar i;
    Sample_And_Smoothing(ADS1x9x_Data_base,ADS1x9x_Data_Smoothing,1);
    for(i=0;i<CHANNEL_SEL;i++)
    {
        if(i==0 || i ==1)
        {
         ADS1x9x_Data_base[i] =0;
        }
        ADS1x9x_Data_base[i] = ADS1x9x_Data_base[i] - 0x017777;
    }
        
}
//***************************************************************************//
//**函数名    ：void Send_Data(char Channel)
//**函数功能  ：将ads298的转换数据发出，去掉单个通道3字节数据的首个字节,并附上导联掉落数据
//**入口参数  ：Channel ，，控制发送数据的通道标号 1~16，发送该标号的单个通道
//**            数据，0，发送所有通道数据
//**返回参数  ：无
//***************************************************************************//
void Send_Data(char Channel)
{

        lastSend(3);
        lastSend(2);
        lastSend(4);
        lastSend(5);
        lastSend(6);
        lastSend(7);
        lastSend(8);
        lastSend(1);
        lastSend(12);
        lastSend(13);
        lastSend(14);
        lastSend(15);
        lastSend(16);
        lastSend(17);


}

//***************************************************************************//
//**函数名    ：lastSend(unsigned char num)
//**函数功能  ：将处理好的数据通过串口发送
//**入口参数  ：num为存储数组的序号
//**返回参数  ：无
//***************************************************************************//
void lastSend(unsigned char num)
{
//   Deal_Data(ADS1x9x_Data[num*3],ADS1x9x_Data[num*3+1],ADS1x9x_Data[num*3+2]);
//               copyToHalfHighByte(shuz,loffP[num],loffN[num]);
//               for(char j=1;j<3;j++)
//               {
//                 while (!(IFG2 & UTXIFG1));
//                 TXBUF1=shuz[2-j];
//                 lsData[num*2+j-1] = shuz[2-j];
//               }
}

unsigned char setAndGetRegister(uchar addr,uchar val)
{
      unsigned char reData;
      Set_ADS1x9x_Chip_Enable();           
      ADS1x9x_SPI_Address_Byte_Count(DEFAULT_WRITE_NUMBER_OF_REGISTERS+addr-1,0x00);
      spi_readChar(val);
      ADS1x9x_SPI_Address_Byte_Count(DEFAULT_READ_NUMBER_OF_REGISTERS+addr-1,0x00);
      reData = spi_readChar(SPI_TEST_DATA);
			Clear_ADS1x9x_Chip_Enable(); 
      return reData;
}
//***************************************************************************//
//**函数名    ：getRegister(unsigned char c)
//**函数功能  ：读取片1一个寄存器值，用作观察
//**入口参数  ：c,为寄存器地址，查手册得到
//**返回参数  ：无
//***************************************************************************//
unsigned char getRegister(unsigned char num)
{
  unsigned char c;
  ADS1x9x_SPI_Address_Byte_Count(DEFAULT_READ_NUMBER_OF_REGISTERS+num-1,0x00);
  c = ADS1x9x_SPI_Burst ( SPI_TEST_DATA);
  Clear_ADS1x9x_Chip_Enable ();  
  return c;
}
//***************************************************************************//
//**函数名    ：getRegister_2(unsigned char c)
//**函数功能  ：读取片2一个寄存器值，用作观察
//**入口参数  ：c,为寄存器地址，查手册得到
//**返回参数  ：无
//***************************************************************************//
unsigned char getRegister_2(unsigned char num)
{
  unsigned char c;
  ADS1x9x_SPI_Address_Byte_Count_2(DEFAULT_READ_NUMBER_OF_REGISTERS+num-1,0x00);
  c = ADS1x9x_SPI_Burst ( SPI_TEST_DATA);
  Clear_ADS1x9x_Chip_Enable_2 ();  
  return c;
}
//***************************************************************************//
//**函数名    ：getLoffP(unsigned char c[])
//**函数功能  ：获得掉落导联P的数据
//**入口参数  ：无
//**返回参数  ：无
//***************************************************************************//
void getLoffP(unsigned char *c)
{
  unsigned lsC;
  lsC = getRegister(0x12);
  for(int i=0;i<8;i++)
  {
    c[i+1] =( lsC & (0x01<<i) )>> i;
  }
  lsC = getRegister_2(0x12);
  for(int i=0;i<8;i++)
  {
    c[i+10] =( lsC & (0x01<<i) )>> i;
  }
}
//***************************************************************************//
//**函数名    ：getLoffN(unsigned char c[])
//**函数功能  ：获得掉落导联N的数据
//**入口参数  ：无
//**返回参数  ：无
//***************************************************************************//
void getLoffN(unsigned char *c)
{
  unsigned lsC;
  lsC = getRegister(0x13);
  for(int i=0;i<8;i++)
  {
    c[i+1] =( lsC & (0x01<<i) )>> i;
  }
  lsC = getRegister_2(0x13);
  for(int i=0;i<8;i++)
  {
    c[i+10] =( lsC & (0x01<<i) )>> i;
  }
}

//***************************************************************************//
//**函数名    ：__interrupt void Timer_A (void)
//**函数功能  ：定时器中断函数
//**入口参数  ：无
//**返回参数  ：无
//***************************************************************************//
void copyToHalfHighByte(unsigned char shuz[],unsigned char loffP,unsigned char loffN)
{
  shuz[1] |= loffP<<7; 
  shuz[1] |= loffN<<6; 
}
//***************************************************************************//
//**函数名    ：tSPI1(void)
//**函数功能  ：任务函数
//**入口参数  ：pvParameters 没有用
//**返回参数  ：无
//**备注			:没有使用
//***************************************************************************//

void tSPI1(void *pvParameters)
{
	//初始化代码
	static uint16_t initFlag = 0;
    
	while(!initFlag)
	{
     
	   init_ADS1x9x3 ();
     set_ADS1x9x_Start_Pin(HIGH); //for rdata指令	 
	   //Double_Chip_Data_get();
     
     
     initFlag = 1;  													  //第一次初始化标志
	}

    //循环代码
	for(;;)
	{
        //Double_Chip_Data_get();    //采集数据
        getLoffP(loffP);         //采集P导联信息
        getLoffN(loffN);         //采集N导联信息
        //Serial_Print_Str("BG");  //字头
         //Send_Data(0);       //发送数据观看
         //Send_Flag = 0;
         Sample_Number++;    //转换次数记录 
	}
}

