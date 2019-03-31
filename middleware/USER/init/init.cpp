#include "include.h"
int Analysis_board_Init(void)
{
	
	
 // 设置中断组为2  4个抢占优先级，4个响应优先级（0-3）
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	
	//关闭计时器时钟，等待初始化完成
	CLOSE_TIME;
	
	//滴答时钟初始化
	Sys_Time.SysTick_Configuration();
	
	//系统计时时钟初始化
	Sys_Time.TIM6_Configuration();
	
	//延时时钟初始化
	while(!delay_init(72));	   
	
	//串口初始化
	while(!apmMAV.Usart_Init(APM_USART,APM_USART_Baud));

	while(!AnalysisUsart.Usart_Init(Analysis_USART,Analysis_USART_Baud));
	
	while(!ReserveUsart.Usart_Init(Reserve_USART,Reserve_USART_Baud));
	
	while(!Flow.Usart_Init(Flow_USART,Flow_USART_Baud));
	//测高初始化
	while(!Sonar.Ultran_Init());
	
	//卡尔曼滤波系数初始化 还有部分图像参数
	Kalman_Filter_Init();
	

//	//PWM输出 50HZ，默认占空比0，4个通道均是
//	while(!PWMOUT.PWMOUT_INIT(50));
//	
//	//自动重装值为0，分频72，以1M频率捕获
//	while(!PWMIN.PWMIN_INIT(0xffff,72-1));
	

	
	//初始化完成后开启计时器
	OPEN_TIME;
	
	return true;
}

