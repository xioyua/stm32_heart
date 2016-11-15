#include "include.h"
#include "BlueTooth.h"
uint8_t str_enter[] = {'\r','\n'};
uint8_t str_at[] = {'a','t'};
uint8_t buffer[30];
uint8_t point_Bak;
//����atָ������ִ����Զ�����\r\n
void SendStr(uint8_t * str,uint8_t n)
{
	uint8_t i;
	for(i=0;i<n;i++)
		HAL_UART_Transmit(&huart1,&str[i],sizeof(str[0]),30);
	HAL_UART_Transmit(&huart1,&str_enter[0],sizeof(str[0]),30);
	HAL_UART_Transmit(&huart1,&str_enter[1],sizeof(str[0]),30);
}
//���ջظ�
void ReceiveBak()
{
	uint8_t temp = HAL_OK;
	uint8_t i ;
	i = 0;
	while(temp == HAL_OK)
  {
        temp = HAL_UART_Receive(&huart1,buffer+i,sizeof(str_at[0]),2000);
        i++;
  }
	point_Bak = --i;
	for(;i<29;i++)
  {
        buffer[i] = 0;
  }
}

typeBlueFlag FindStr(uint8_t *str,uint8_t num)
{
	uint8_t i,j,n;
	n = num;
	for(i=0;i<point_Bak;i++)
	{
		for(j=0;j<n;j++)
		{
			if(buffer[i+j] != str[j])
			{
				break;
			}
			//�������str�����һ���ַ�
			if(j==n-1)
			{
			  return BLUE_OK;
			}
		}
	}
			
	
	return BLUE_ERROR;
}


//������в��������Ƿ���ȷ
typeBlueFlag CheckPara()
{
    typeBlueFlag status=BLUE_OK;

    SendStr(str_at,2);			//AT����
	ReceiveBak();	            //���ջش�
	status = FindStr("OK",2);
    if(status == BLUE_OK)
    {
        SendStr("at+name?",8);			//name����
	    ReceiveBak();	            //���ջش�
	    status = FindStr("+NAME:ecg",9);
	    if(status == BLUE_OK)
	    {
            SendStr("at+uart?",8);			//�����ʲ���
	        ReceiveBak();	            //���ջش�
	        status = FindStr("+UART:38400,0,0",15);
	        if(status == BLUE_OK)
	        {
                SendStr("at+role?",8);			//role����
    	        ReceiveBak();	            //���ջش�
    	        status = FindStr("+ROLE:0",7);
    	        if(status == BLUE_OK)
    	        {
                    SendStr("at+cmode?",9);			//cmode����
        	        ReceiveBak();	            //���ջش�
        	        status = FindStr("+CMOD:1",7);
        	        if(status == BLUE_OK)
        	        {
        	            return BLUE_OK;
        	        }
        	        else return BLUE_ERROR6;
    	        
    	        }
    	        else return BLUE_ERROR5;
	        }
	        else return BLUE_ERROR4;
	    }
	    else return BLUE_ERROR3;
	        
	}
	else
	{
	    return BLUE_ERROR2;
	}

	
}

//������������
typeBlueFlag SetPara()
{
    typeBlueFlag status=BLUE_OK;

    SendStr(str_at,2);			//AT����
	ReceiveBak();	            //���ջش�
	status = FindStr("OK",2);
    if(status == BLUE_OK)
    {
        SendStr("at+name=ecg",11);	//name����
	    ReceiveBak();	            //���ջش�
	    status = FindStr("OK",2);
	    if(status == BLUE_OK)
	    {
            SendStr("at+uart=38400,0,0",17); //�����ʲ���
	        ReceiveBak();	            //���ջش�
	        status = FindStr("OK",2);
	        if(status == BLUE_OK)
	        {
                SendStr("at+role=0",9);		//role����
    	        ReceiveBak();	            //���ջش�
    	        status = FindStr("OK",2);
    	        if(status == BLUE_OK)
    	        {
                    SendStr("at+cmode=1",10);			//cmode����
        	        ReceiveBak();	            //���ջش�
        	        status = FindStr("OK",2);
        	        if(status == BLUE_OK)
        	        {
        	            return BLUE_OK;
        	        }
        	        else return BLUE_ERROR6;
    	        
    	        }
    	        else return BLUE_ERROR5;
	        }
	        else return BLUE_ERROR4;
	    }
	    else return BLUE_ERROR3;
	        
	}
	else
	{
	    return BLUE_ERROR2;
	}

}
void BuleTooth_Init()
{
	typeBlueFlag status;
	//SET_BS_Key;//��atģʽ
	
// 	delay_ms(1000);
// 	SendStr(str_at,2);			//����ָ��
// 	ReceiveBak();	//���ջش�
// 	status = FindStr("OK",2);
// 	
// 	SendStr("at+init",7);	//��ʼ������
// 	ReceiveBak();       
// 	delay_ms(2000);
	if(gBlueToothSetFlag)
	{
	  SET_BS_Key;//��atģʽ
	  delay_ms(300);
	status = CheckPara();
	while(status != BLUE_OK)
	{
	    LEDGreen_CLEAR;
	    LEDRed_SET;
      status = SetPara();
	    status = CheckPara();
	}
    LEDGreen_SET;
	SendStr("at+reset",8);
	delay_ms(100);
}
	
	CLR_BS_Key;//�ر�atģʽ
  USART1->BRR = UART_BRR_SAMPLING8(HAL_RCC_GetPCLK2Freq(),230400);     
	
    
    
    
}
