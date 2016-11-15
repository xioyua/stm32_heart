#include "include.h"
/* Private define ------------------------------------------------------------*/
//WRITER:��Դ

/* Private macro -------------------------------------------------------------*/
/* ----------------------------------------���� ---------------------------------------------------------*/

volatile uint32_t  ADS1x9x_Data [CHANNEL_SEL];  				//ADS1298ͨ�����ݻ���
uint8_t  ADS1x9x_Data_Smoothing[ADS1298_DATA_LENGTH]; 	//���������˲��Ļ���
volatile unsigned char ADS1298_lsData[ADS1298_LASTDATA_LWNGTH];//���ڴ�����Ҫ���͵�����
int32_t      ADS1x9x_Data_base[CHANNEL_SEL];
unsigned char Serial_Buffer=0;                     //�жϴ�������ı�־
unsigned int  Timer_Flag =0;                       //ʱ�ӱ�־
unsigned char ADS1298_ON_OFF=1;                    //����
unsigned int  Sample_Number=0;                      //��¼ת������
unsigned char Send_Flag=1;                         //�Լ����Ƶ�ת��ʹ�ܱ�־
unsigned char shuz[2];                             //���Ӻ���Deal_Data��ʹ��

unsigned char LSRegister;
unsigned char loffP[18];
unsigned char loffN[18];

/* ------------------------------------------�������� -----------------------------------------------*/


char		  Double_Chip_Data_get(uint8_t * sumBuf);      //��оƬ���ݲɼ�
void 					Send_Data(char Channel);         //���ڷ�������
// void 					DMA_Init();
unsigned char getRegister(unsigned char num);  //��ȡ�Ĵ�����ֵ
void 					getLoffP(unsigned char *c);
void 					getLoffN(unsigned char *c);
void 					copyToHalfHighByte(unsigned char shuz[],unsigned char loffP,unsigned char loffN);
void 					lastSend(unsigned char num);      //������������ͨ�����ڷ���
/*------------------------------��������---------------------------------------------*/
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
//**������    ��void Start_Sample(void)
//**��������  �����Ͳɼ�������ɼ��ͽ��ֿܷ����м�ĵȴ�ʱ������������д��ڴ���
//**��ڲ���  ����
//**���ز���  ����
//**���ݴ洢ȫ�ֱ��� : ADS1x9x_Data[ADS1298_DATA_LENGTH]
//***************************************************************************//
void Start_Sample_Mannully()
{
  CLR_ADS1298_CS;
  #if CHANNEL_SEL == CHANNEL16
  CLR_ADS1298_CS_2;
  #endif
  delay_n(20);
  spi_readChar( READ_DATA_MANUALLY);//�ֶ�������
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
  spi_readChar( SET_READ_DATA_CONTINUOUSLY);//����������ģʽ
  SET_ADS1298_CS;
  SET_ADS1298_CS_2;
}
//***************************************************************************//
//**������    ��void Double_Chip_Data_get(void)
//**��������  ������оƬ���ϲɼ����ݣ���54�ֽڣ���������48�ֽڣ�16ͨ��
//**��ڲ���  ����
//**���ز���  ����
//**���ݴ洢ȫ�ֱ��� : ADS1x9x_Data[ADS1298_DATA_LENGTH]
//***************************************************************************//
char Double_Chip_Data_get(uint8_t  sumBuf[])
{
  unsigned char i = 0 ;
  
  
  
    {
        //�ȴ�
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
//**������    ��void Deal_Data(char data)
//**��������  ������������4λ�����λ����
//**��ڲ���  ������
//**���ز���  ����
//***************************************************************************//
#if FORMAT == BIT12
void Deal_Data()
{
   unsigned char i;
   //��ͷ
   ADS1298_lsData[0]='B';
   ADS1298_lsData[1]='G';

   //ͨ��2˳���2
   ADS1298_lsData[4] = (ADS1x9x_Data[1] &0x00f000)>>12;
   ADS1298_lsData[5] = ((ADS1x9x_Data[1] & 0x000ff0 )>>4) ;
   //ͨ��3˳��1
   ADS1298_lsData[2] = (ADS1x9x_Data[2] &0x00f000)>>12;
   ADS1298_lsData[3] = ((ADS1x9x_Data[2] & 0x000ff0 )>>4) ;
   //ͨ��4-8
   for(i=3;i<=7;i++)
   {
        ADS1298_lsData[(i)*2] = (ADS1x9x_Data[i] &0x00f000)>>12;
        ADS1298_lsData[(i)*2+1] = ((ADS1x9x_Data[i] & 0x000ff0 )>>4) ;
   }
   //ͨ��1
   ADS1298_lsData[16]=(ADS1x9x_Data[0] &0x00f000)>>12;
   ADS1298_lsData[17]=((ADS1x9x_Data[0] & 0x000ff0 )>>4) ;
   #if CHANNEL_SEL == CHANNEL16
   //ͨ��12-16
    for(i=9;i<=13;i++)
    {
        ADS1298_lsData[(i)*2] = (ADS1x9x_Data[i+2] &0x00f000)>>12;
        ADS1298_lsData[(i)*2+1] =((ADS1x9x_Data[i+2] & 0x000ff0 )>>4);
    }
    //ͨ��11
    ADS1298_lsData[28]=(ADS1x9x_Data[10] &0x00f000)>>12;
    ADS1298_lsData[29]=((ADS1x9x_Data[10] & 0x000ff0 )>>4) ;
   #endif


}
#elif FORMAT == BIT16

void Deal_Data()
{
   unsigned char i;
   
   //ͨ��2-8
   for(i=0;i<=6;i++)
   {
        ADS1298_lsData[(i)*2+1] =  (ADS1x9x_Data[(i+1)]&0xff00)>>8;  //��λ
        ADS1298_lsData[(i)*2] = (ADS1x9x_Data[i+1]&0x00ff);         //��λ
        ADS1298_lsData[(i)*2+1] = (0x80 & ADS1298_lsData[(i)*2+1]) | ((0x3e & ADS1298_lsData[(i)*2+1]) <<1)|((ADS1298_lsData[(i)*2]&0x80)>>7);
        ADS1298_lsData[(i)*2] = (ADS1298_lsData[(i)*2] & 0x7f)<<1;
   }
   //ͨ��1
   ADS1298_lsData[15] =  (ADS1x9x_Data[(0)]&0x7f00)>>8;
   ADS1298_lsData[14] = (ADS1x9x_Data[0]&0x00ff);
   ADS1298_lsData[15] = (0x80 & ADS1298_lsData[15]) | ((0x3e & ADS1298_lsData[15])<<1) |((ADS1298_lsData[14]&0x80)>>7);
   ADS1298_lsData[14] = (ADS1298_lsData[14] & 0x7f)<<1;

   #if CHANNEL_SEL == CHANNEL16
   //ͨ��12-16
    for(i=8;i<=12;i++)
    {
        ADS1298_lsData[(i)*2+1] = ADS1x9x_Data[(i+5)*3] + 0x80;
        ADS1298_lsData[(i)*2] = ADS1x9x_Data[(i+5)*3+1];
    }
    //ͨ��11
    ADS1298_lsData[27]=ADS1x9x_Data[36] + 0x80;
    ADS1298_lsData[26]=ADS1x9x_Data[37];
   #endif

}
#endif
//***************************************************************************//
//**������    ��void Sample_And_Smoothing(uint32_t* distination,uint8_t* sumBuf,uchar num)
//**��������  ���ɼ�������
//**��ڲ���  ������
//**���ز���  ����
//***************************************************************************//

void Sample_And_Smoothing(uint32_t* distination,uint8_t* sumBuf,uchar num)
{
    uchar i;
    //��βɼ���ÿ��ͨ�������ݶ���������Ӧ��sumBuf��
    for(i=0;i<CHANNEL_SEL;i++)
	{
			distination[i] = 0;
	}
        //set_ADS1x9x_Start_Pin(LOW);               //�õ�STARTpin
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
       // set_ADS1x9x_Start_Pin(HIGH);               //�õ�STARTpin
    
    //��ƽ��ֵ�������ADS1x9x_Data��
    
		
    
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
//ȥ���ߺ�����
//		���ݵ�һ��ȡֵ���õ����������ֵ(0x017777)֮��Ĳ���֮���ȡ�������У����Զ���ȥ�����ֵ
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
//**������    ��void Send_Data(char Channel)
//**��������  ����ads298��ת�����ݷ�����ȥ������ͨ��3�ֽ����ݵ��׸��ֽ�,�����ϵ�����������
//**��ڲ���  ��Channel �������Ʒ������ݵ�ͨ����� 1~16�����͸ñ�ŵĵ���ͨ��
//**            ���ݣ�0����������ͨ������
//**���ز���  ����
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
//**������    ��lastSend(unsigned char num)
//**��������  ��������õ�����ͨ�����ڷ���
//**��ڲ���  ��numΪ�洢��������
//**���ز���  ����
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
//**������    ��getRegister(unsigned char c)
//**��������  ����ȡƬ1һ���Ĵ���ֵ�������۲�
//**��ڲ���  ��c,Ϊ�Ĵ�����ַ�����ֲ�õ�
//**���ز���  ����
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
//**������    ��getRegister_2(unsigned char c)
//**��������  ����ȡƬ2һ���Ĵ���ֵ�������۲�
//**��ڲ���  ��c,Ϊ�Ĵ�����ַ�����ֲ�õ�
//**���ز���  ����
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
//**������    ��getLoffP(unsigned char c[])
//**��������  ����õ��䵼��P������
//**��ڲ���  ����
//**���ز���  ����
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
//**������    ��getLoffN(unsigned char c[])
//**��������  ����õ��䵼��N������
//**��ڲ���  ����
//**���ز���  ����
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
//**������    ��__interrupt void Timer_A (void)
//**��������  ����ʱ���жϺ���
//**��ڲ���  ����
//**���ز���  ����
//***************************************************************************//
void copyToHalfHighByte(unsigned char shuz[],unsigned char loffP,unsigned char loffN)
{
  shuz[1] |= loffP<<7; 
  shuz[1] |= loffN<<6; 
}
//***************************************************************************//
//**������    ��tSPI1(void)
//**��������  ��������
//**��ڲ���  ��pvParameters û����
//**���ز���  ����
//**��ע			:û��ʹ��
//***************************************************************************//

void tSPI1(void *pvParameters)
{
	//��ʼ������
	static uint16_t initFlag = 0;
    
	while(!initFlag)
	{
     
	   init_ADS1x9x3 ();
     set_ADS1x9x_Start_Pin(HIGH); //for rdataָ��	 
	   //Double_Chip_Data_get();
     
     
     initFlag = 1;  													  //��һ�γ�ʼ����־
	}

    //ѭ������
	for(;;)
	{
        //Double_Chip_Data_get();    //�ɼ�����
        getLoffP(loffP);         //�ɼ�P������Ϣ
        getLoffN(loffN);         //�ɼ�N������Ϣ
        //Serial_Print_Str("BG");  //��ͷ
         //Send_Data(0);       //�������ݹۿ�
         //Send_Flag = 0;
         Sample_Number++;    //ת��������¼ 
	}
}

