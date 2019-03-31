#include "include.h"
PWMIN_CONF  PWMIN;
// PWM输入捕获初始化
int PWMIN_CONF::PWMIN_INIT(UINT_16 arr,UINT_16 psc)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
  TIM_ICInitTypeDef TIM4_ICInitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);	//使能TIM4时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);  //使能GPIOB时钟

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_8
			| GPIO_Pin_9;  //PB6,7,8,9 清除之前设置  
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD; //PB6,7,8,9 输入 
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	GPIO_ResetBits(GPIOB, GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9);//PB6,7,8,9  下拉

	//初始化定时器4 TIM4	 
	TIM_TimeBaseStructure.TIM_Period = arr; //设定计数器自动重装值 
	TIM_TimeBaseStructure.TIM_Prescaler = psc; 	//预分频器 
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; //设置时钟分割:TDTS = Tck_tim
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //TIM向上计数模式
	TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure); //根据TIM_TimeBaseInitStruct中指定的参数初始化TIMx的时间基数单位

	//初始化TIM4输入捕获参数 通道1
	TIM4_ICInitStructure.TIM_Channel = TIM_Channel_1; //CC1S=01 	选择输入端 IC1映射到TI1上
	TIM4_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;	//上升沿捕获
	TIM4_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI; //映射到TI1上
	TIM4_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;	  //配置输入分频,不分频 
	TIM4_ICInitStructure.TIM_ICFilter = 0x00;	  //IC1F=0000 配置输入滤波器 不滤波
	TIM_ICInit(TIM4, &TIM4_ICInitStructure);

	//初始化TIM4输入捕获参数 通道2
	TIM4_ICInitStructure.TIM_Channel = TIM_Channel_2; //CC1S=01 	选择输入端 IC1映射到TI1上
	TIM4_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;	//上升沿捕获
	TIM4_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI; //映射到TI1上
	TIM4_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;	  //配置输入分频,不分频 
	TIM4_ICInitStructure.TIM_ICFilter = 0x00;	  //IC1F=0000 配置输入滤波器 不滤波
	TIM_ICInit(TIM4, &TIM4_ICInitStructure);

	//初始化TIM4输入捕获参数 通道3
	TIM4_ICInitStructure.TIM_Channel = TIM_Channel_3; //CC1S=01 	选择输入端 IC1映射到TI1上
	TIM4_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;	//上升沿捕获
	TIM4_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI; //映射到TI1上
	TIM4_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;	  //配置输入分频,不分频 
	TIM4_ICInitStructure.TIM_ICFilter = 0x00;	  //IC1F=0000 配置输入滤波器 不滤波
	TIM_ICInit(TIM4, &TIM4_ICInitStructure);

	//初始化TIM4输入捕获参数 通道4
	TIM4_ICInitStructure.TIM_Channel = TIM_Channel_4; //CC1S=01 	选择输入端 IC1映射到TI1上
	TIM4_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;	//上升沿捕获
	TIM4_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI; //映射到TI1上
	TIM4_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;	  //配置输入分频,不分频 
	TIM4_ICInitStructure.TIM_ICFilter = 0x00;	  //IC1F=0000 配置输入滤波器 不滤波
	TIM_ICInit(TIM4, &TIM4_ICInitStructure);

	
	NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn;  //TIM4中断
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;  //先占优先级2级
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = PWMIN_NVIC;  //从优先级0级
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; //IRQ通道被使能
	NVIC_Init(&NVIC_InitStructure);   //根据NVIC_InitStruct中指定的参数初始化外设NVIC寄存器 

	TIM_ITConfig(TIM4, TIM_IT_CC1 | TIM_IT_CC2 | TIM_IT_CC3 | TIM_IT_CC4,ENABLE);   //不允许更新中断，允许CC1IE,CC2IE,CC3IE,CC4IE捕获中断	

	TIM_Cmd(TIM4, ENABLE); 		//使能定时器4
	
	return 1;
	
}
void PWMIN_CONF::PWMIN_Intterupt(void)
{

if ((TIM4CH1_CAPTURE_STA & 0X80) == 0) 		//还未成功捕获	
	{
		if (TIM_GetITStatus(TIM4, TIM_IT_CC1) != RESET) 		//捕获1发生捕获事件
		{
			TIM_ClearITPendingBit(TIM4, TIM_IT_CC1); 		//清除中断标志位
			if (TIM4CH1_CAPTURE_STA & 0X40)		//捕获到一个下降沿
			{
				TIM4CH1_CAPTURE_DOWNVAL = TIM_GetCapture1(TIM4);//记录下此时的定时器计数值
				if (TIM4CH1_CAPTURE_DOWNVAL < TIM4CH1_CAPTURE_UPVAL)
				{
					tim4_T1 = 65535;
				}
				else
					tim4_T1 = 0;
				PWMIN.pwmWidthCH1 = TIM4CH1_CAPTURE_DOWNVAL - TIM4CH1_CAPTURE_UPVAL+ tim4_T1;		//得到总的高电平的时间
//				pwmout1 = tempup1;		//总的高电平的时间
				TIM4CH1_CAPTURE_STA = 0;		//捕获标志位清零
				TIM_OC1PolarityConfig(TIM4, TIM_ICPolarity_Rising); //设置为上升沿捕获		  
			}
			else //发生捕获时间但不是下降沿，第一次捕获到上升沿，记录此时的定时器计数值
			{
				TIM4CH1_CAPTURE_UPVAL = TIM_GetCapture1(TIM4);		//获取上升沿数据
				TIM4CH1_CAPTURE_STA |= 0X40;		//标记已捕获到上升沿
				TIM_OC1PolarityConfig(TIM4, TIM_ICPolarity_Falling);//设置为下降沿捕获
			}
		}
	}

	if ((TIM4CH2_CAPTURE_STA & 0X80) == 0)		//还未成功捕获	
	{
		if (TIM_GetITStatus(TIM4, TIM_IT_CC2) != RESET)		//捕获2发生捕获事件
		{
			TIM_ClearITPendingBit(TIM4, TIM_IT_CC2);		//清除中断标志位
			if (TIM4CH2_CAPTURE_STA & 0X40)		//捕获到一个下降沿
			{
				TIM4CH2_CAPTURE_DOWNVAL = TIM_GetCapture2(TIM4);//记录下此时的定时器计数值
				if (TIM4CH2_CAPTURE_DOWNVAL < TIM4CH2_CAPTURE_UPVAL)
				{
					tim4_T2 = 65535;
				}
				else
					tim4_T2 = 0;
				PWMIN.pwmWidthCH2 = TIM4CH2_CAPTURE_DOWNVAL - TIM4CH2_CAPTURE_UPVAL+ tim4_T2;		//得到总的高电平的时间
//				pwmout2 = tempup2;		//总的高电平的时间
				TIM4CH2_CAPTURE_STA = 0;		//捕获标志位清零
				TIM_OC2PolarityConfig(TIM4, TIM_ICPolarity_Rising); //设置为上升沿捕获		  
			}
			else //发生捕获时间但不是下降沿，第一次捕获到上升沿，记录此时的定时器计数值
			{
				TIM4CH2_CAPTURE_UPVAL = TIM_GetCapture2(TIM4);		//获取上升沿数据
				TIM4CH2_CAPTURE_STA |= 0X40;		//标记已捕获到上升沿
				TIM_OC2PolarityConfig(TIM4, TIM_ICPolarity_Falling);//设置为下降沿捕获
			}
		}
	}

	if ((TIM4CH3_CAPTURE_STA & 0X80) == 0)		//还未成功捕获	
	{
		if (TIM_GetITStatus(TIM4, TIM_IT_CC3) != RESET)		//捕获3发生捕获事件
		{
			TIM_ClearITPendingBit(TIM4, TIM_IT_CC3);		//清除中断标志位
			if (TIM4CH3_CAPTURE_STA & 0X40)		//捕获到一个下降沿
			{
				TIM4CH3_CAPTURE_DOWNVAL = TIM_GetCapture3(TIM4);//记录下此时的定时器计数值
				if (TIM4CH3_CAPTURE_DOWNVAL < TIM4CH3_CAPTURE_UPVAL)
				{
					tim4_T3 = 65535;
				}
				else
					tim4_T3 = 0;
				PWMIN.pwmWidthCH3 = TIM4CH3_CAPTURE_DOWNVAL - TIM4CH3_CAPTURE_UPVAL+ tim4_T3;		//得到总的高电平的时间
//				pwmout3 = tempup3;		//总的高电平的时间
				TIM4CH3_CAPTURE_STA = 0;		//捕获标志位清零
				TIM_OC3PolarityConfig(TIM4, TIM_ICPolarity_Rising); //设置为上升沿捕获		  
			}
			else //发生捕获时间但不是下降沿，第一次捕获到上升沿，记录此时的定时器计数值
			{
				TIM4CH3_CAPTURE_UPVAL = TIM_GetCapture3(TIM4);		//获取上升沿数据
				TIM4CH3_CAPTURE_STA |= 0X40;		//标记已捕获到上升沿
				TIM_OC3PolarityConfig(TIM4, TIM_ICPolarity_Falling);//设置为下降沿捕获
			}
		}
	}

	if ((TIM4CH4_CAPTURE_STA & 0X80) == 0)		//还未成功捕获	
	{
		if (TIM_GetITStatus(TIM4, TIM_IT_CC4) != RESET)		//捕获4发生捕获事件
		{
			TIM_ClearITPendingBit(TIM4, TIM_IT_CC4);		//清除中断标志位
			if (TIM4CH4_CAPTURE_STA & 0X40)		//捕获到一个下降沿
			{
				TIM4CH4_CAPTURE_DOWNVAL = TIM_GetCapture4(TIM4);//记录下此时的定时器计数值
				if (TIM4CH4_CAPTURE_DOWNVAL < TIM4CH4_CAPTURE_UPVAL)
				{
					tim4_T4 = 65535;
				}
				else
					tim4_T4 = 0;
				PWMIN.pwmWidthCH4 = TIM4CH4_CAPTURE_DOWNVAL - TIM4CH4_CAPTURE_UPVAL+ tim4_T4;		//得到总的高电平的时间
//				pwmout4 = tempup4;		//总的高电平的时间
				TIM4CH4_CAPTURE_STA = 0;		//捕获标志位清零
				TIM_OC4PolarityConfig(TIM4, TIM_ICPolarity_Rising); //设置为上升沿捕获		  
			}
			else //发生捕获时间但不是下降沿，第一次捕获到上升沿，记录此时的定时器计数值
			{
				TIM4CH4_CAPTURE_UPVAL = TIM_GetCapture4(TIM4);		//获取上升沿数据
				TIM4CH4_CAPTURE_STA |= 0X40;		//标记已捕获到上升沿
				TIM_OC4PolarityConfig(TIM4, TIM_ICPolarity_Falling);//设置为下降沿捕获
			}
		}
	}
}




