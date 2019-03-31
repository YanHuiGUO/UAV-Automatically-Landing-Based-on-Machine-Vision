#include "include.h"
TimeGroup TimeSleep;
static volatile float Cycle_T[GET_TIME_NUM][3];
volatile UINT_32 sysTickUptime ;
SYSTIME_CONF Sys_Time;
void  SYSTIME_CONF::SysTick_Configuration(void)
{
	RCC_ClocksTypeDef  rcc_clocks;
	uint32_t         cnts;

	RCC_GetClocksFreq(&rcc_clocks);

	cnts = (uint32_t)rcc_clocks.HCLK_Frequency / TICK_PER_SECOND;
	cnts = cnts / 8;

	SysTick_CLKSourceConfig(SysTick_CLKSource_HCLK_Div8);//8分频时钟源，72M->9M
	SysTick_Config(cnts);//9000,分频到1K频率计数

}
static UINT_8 sysTickUptime_OVER =0 ;//计数变量溢出标志位
UINT_64  SYSTIME_CONF::GetSysTime_us(void) 
{
	register uint32_t ms;
	UINT_64 value;
	ms = sysTickUptime;
	if(sysTickUptime>=0xffffffff)//1193个小时后才会溢出
		sysTickUptime_OVER = 1;
	if(sysTickUptime_OVER == 1)
	{
			value = 0xffffffff+ ms * TICK_US + (SysTick->LOAD - SysTick->VAL) * TICK_US / SysTick->LOAD;
			sysTickUptime_OVER = 0;
	}
	value = ms * TICK_US + (SysTick->LOAD - SysTick->VAL) * TICK_US / SysTick->LOAD;
	return value;
}

enum
{
	OLD =0,
	NOW =1,
	NEW =2
};
FLOAT_32  SYSTIME_CONF::Get_Cycle_T(u8 item)	
{
	Cycle_T[item][OLD] = Cycle_T[item][NOW];	//上一次的时间
	Cycle_T[item][NOW] = GetSysTime_us()/1000000.0f; //本次的时间
	Cycle_T[item][NEW] = ( ( Cycle_T[item][NOW] - Cycle_T[item][OLD] ) );//间隔的时间（周期）
	return Cycle_T[item][NEW];
}

void  SYSTIME_CONF::TIM6_Configuration(void)
{
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
		
		// 开启TIMx_CLK,x[6,7],即内部时钟CK_INT=72M
    RCC_APB1PeriphClockCmd (RCC_APB1Periph_TIM6, ENABLE);
	
		// 自动重装载寄存器周的值(计数值)
    TIM_TimeBaseStructure.TIM_Period=1000;
	
    // 累计 TIM_Period个频率后产生一个更新或者中断
	  // 时钟预分频数为71，则驱动计数器的时钟CK_CNT = CK_INT / (71+1)=1M
    TIM_TimeBaseStructure.TIM_Prescaler= 71;
	
		// 时钟分频因子 ，基本定时器TIM6和TIM7没有，不用管
    //TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1;
		
		// 计数器计数模式，基本定时器TIM6和TIM7只能向上计数，没有计数模式的设置，不用管
    //TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up; 
		
		// 重复计数器的值，基本定时器TIM6和TIM7没有，不用管
		//TIM_TimeBaseStructure.TIM_RepetitionCounter=0;
	
	  // 初始化定时器TIMx, x[6,7]
    TIM_TimeBaseInit(TIM6, &TIM_TimeBaseStructure);
		
		// 清除计数器中断标志位
    TIM_ClearFlag(TIM6, TIM_FLAG_Update);
	  
		// 开启计数器中断
    TIM_ITConfig(TIM6,TIM_IT_Update,ENABLE);
		
		// 使能计数器
    TIM_Cmd(TIM6, ENABLE);																		
  

    NVIC_InitTypeDef NVIC_InitStructure; 
		
		// 设置中断来源
    NVIC_InitStructure.NVIC_IRQChannel = TIM6_IRQn;
	
		// 设置主优先级为 0
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	 
	  // 设置抢占优先级为0//低于超声波
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;	
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
	
}
void SYSTIME_CONF::Duty_Loop(void)   					//最短任务周期为1ms，总的代码执行时间需要小于1ms。
{

	if( loop.check_flag == 1 )
	{
			Duty_1ms();							//周期1ms的任务
		
		if( loop.cnt_2ms >= 2 )
		{
			loop.cnt_2ms = 0;
			Duty_2ms();						//周期2ms的任务
		}
		if( loop.cnt_5ms >= 5 )
		{
			loop.cnt_5ms = 0;
			Duty_5ms();						//周期5ms的任务
		}
		if( loop.cnt_10ms >= 10 )
		{
			loop.cnt_10ms = 0;
			Duty_10ms();					//周期10ms的任务
		}
		if( loop.cnt_20ms >= 20 )
		{
			loop.cnt_20ms = 0;
			Duty_20ms();					//周期20ms的任务
		}
		if( loop.cnt_50ms >= 50 )
		{
			loop.cnt_50ms = 0;
			Duty_50ms();					//周期50ms的任务
		}
		if( loop.cnt_100ms >= 100 )
		{
			loop.cnt_100ms = 0;
			Duty_100ms();					//周期1000ms的任务
		}
		if( loop.cnt_500ms >= 500 )
		{
			loop.cnt_500ms = 0;
			Duty_500ms();					//周期1000ms的任务
		}
		loop.check_flag = 0;		//循环运行完毕标志
	}
}

void SYSTIME_CONF::Duty_1ms()
{
		ANO_DT_Data_Exchange();
	//myt = Sys_Time.Get_Cycle_T(1);
		// DMA传数据	


//		for(int i=0;i<PC_BUFFER_LEN;i++)
//			SEND2_PC[i] = 0xaa;
//		if(mavMsgFlag.Flag_sendToPC == 0)
//		{
//				DMA_SetCurrDataCounter(DMA1_Channel4,PC_BUFFER_LEN);	
//				DMA_Cmd(DMA1_Channel4, ENABLE);
//				mavMsgFlag.Flag_sendToPC = 1;	
//		}
		
//		AnalysisUsart.generatePacket_ToAPM(SEND2_APM,APMLEN);
//	for(int i=0;i<APMLEN;i++)
//		AnalysisUsart.Analysis_Send(SEND2_APM[i]);
	
}
void SYSTIME_CONF::Duty_2ms()
{
	
}
void SYSTIME_CONF::Duty_5ms()
{
//	for (int i = 0;i < APMLEN;i++)   
//				SEND2_APM[i] = 0x50; 
		AnalysisUsart.generatePacket_ToAPM(SEND2_APM,Frame_Len);
		if(mavMsgFlag.Flag_sendToAPM == 0)
		{			
				DMA_SetCurrDataCounter(DMA1_Channel2,APMLEN);	
				DMA_Cmd(DMA1_Channel2, ENABLE);
				mavMsgFlag.Flag_sendToAPM = 1;			
		}
		
}
void SYSTIME_CONF::Duty_10ms()
{
		apmMAV.Target_V_Get(0,0,0.01);
}

void SYSTIME_CONF::Duty_20ms()
{
		Sonar.sonar_read(&Sonar.UltraHeight,&Sonar.UltraHeight_raw);
}

void SYSTIME_CONF::Duty_50ms()
{
		Sonar.Ultra_Ranging();
		apmMAV.Target_VisonData_Deal(0.05);
}

void SYSTIME_CONF::Duty_100ms()
{
	

}
void SYSTIME_CONF::Duty_500ms()
{
	
}
void SYSTIME_CONF::Loop_check(void)  //TIME INTTERRUPT
{
	loop.time++; //u16
	loop.cnt_2ms++;
	loop.cnt_5ms++;
	loop.cnt_10ms++;
	loop.cnt_20ms++;
	loop.cnt_50ms++;
	loop.cnt_100ms++;
	loop.cnt_500ms++;
	if( loop.check_flag == 1)
	{
		loop.err_flag ++;     //每累加一次，证明代码在预定周期内没有跑完。
	}
	else
	{	
		loop.check_flag = 1;	//该标志位在循环的最后被清零
	}
}