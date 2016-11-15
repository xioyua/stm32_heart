#include "include.h"
#include "BlueTooth.h"
uint8_t str_enter[] = {'\r','\n'};
uint8_t str_at[] = {'a','t'};
uint8_t buffer[30];
uint8_t point_Bak;
//发送at指令，发送字串后自动附加\r\n
void SendStr(uint8_t * str,uint8_t n)
{
	uint8_t i;
	for(i=0;i<n;i++)
		HAL_UART_Transmit(&huart1,&str[i],sizeof(str[0]),30);
	HAL_UART_Transmit(&huart1,&str_enter[0],sizeof(str[0]),30);
	HAL_UART_Transmit(&huart1,&str_enter[1],sizeof(str[0]),30);
}
//接收回复
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
			//如果这是str中最后一个字符
			if(j==n-1)
			{
			  return BLUE_OK;
			}
		}
	}
			
	
	return BLUE_ERROR;
}


//检查所有参数设置是否正确
typeBlueFlag CheckPara()
{
    typeBlueFlag status=BLUE_OK;

    SendStr(str_at,2);			//AT测试
	ReceiveBak();	            //接收回答
	status = FindStr("OK",2);
    if(status == BLUE_OK)
    {
        SendStr("at+name?",8);			//name测试
	    ReceiveBak();	            //接收回答
	    status = FindStr("+NAME:ecg",9);
	    if(status == BLUE_OK)
	    {
            SendStr("at+uart?",8);			//波特率测试
	        ReceiveBak();	            //接收回答
	        status = FindStr("+UART:38400,0,0",15);
	        if(status == BLUE_OK)
	        {
                SendStr("at+role?",8);			//role测试
    	        ReceiveBak();	            //接收回答
    	        status = FindStr("+ROLE:0",7);
    	        if(status == BLUE_OK)
    	        {
                    SendStr("at+cmode?",9);			//cmode测试
        	        ReceiveBak();	            //接收回答
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

//设置蓝牙参数
typeBlueFlag SetPara()
{
    typeBlueFlag status=BLUE_OK;

    SendStr(str_at,2);			//AT测试
	ReceiveBak();	            //接收回答
	status = FindStr("OK",2);
    if(status == BLUE_OK)
    {
        SendStr("at+name=ecg",11);	//name设置
	    ReceiveBak();	            //接收回答
	    status = FindStr("OK",2);
	    if(status == BLUE_OK)
	    {
            SendStr("at+uart=38400,0,0",17); //波特率测试
	        ReceiveBak();	            //接收回答
	        status = FindStr("OK",2);
	        if(status == BLUE_OK)
	        {
                SendStr("at+role=0",9);		//role测试
    	        ReceiveBak();	            //接收回答
    	        status = FindStr("OK",2);
    	        if(status == BLUE_OK)
    	        {
                    SendStr("at+cmode=1",10);			//cmode测试
        	        ReceiveBak();	            //接收回答
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
	//SET_BS_Key;//打开at模式
	
// 	delay_ms(1000);
// 	SendStr(str_at,2);			//发送指令
// 	ReceiveBak();	//接收回答
// 	status = FindStr("OK",2);
// 	
// 	SendStr("at+init",7);	//初始化蓝牙
// 	ReceiveBak();       
// 	delay_ms(2000);
	if(gBlueToothSetFlag)
	{
	  SET_BS_Key;//打开at模式
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
	
	CLR_BS_Key;//关闭at模式
  USART1->BRR = UART_BRR_SAMPLING8(HAL_RCC_GetPCLK2Freq(),230400);     
	
    
    
    
}
