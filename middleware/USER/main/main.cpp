#include "include.h"
int main(void)
{
	//板子初始化
	Analysis_board_Init();
	while(1)
	{
		//主循环
			Sys_Time.Duty_Loop();

//		delay_ms(10); 
//		Soner.Ultra_TX(US100);
//		Soner.Dac1_Set_Vol(0);
//		printf("Channel 1 : %d us\r\n",	PWMIN.pwmWidthCH1);
//		delay_ms(10);
//		printf("Channel 2 : %d us\r\n",	PWMIN.pwmWidthCH2);
//		delay_ms(10);
//		printf("Channel 3 : %d us\r\n",	PWMIN.pwmWidthCH3);
//		delay_ms(10);
//		printf("Channel 4 : %d us\r\n",	PWMIN.pwmWidthCH4);
	}	 
	return 0;
}

