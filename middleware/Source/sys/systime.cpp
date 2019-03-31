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

	SysTick_CLKSourceConfig(SysTick_CLKSource_HCLK_Div8);//8��Ƶʱ��Դ��72M->9M
	SysTick_Config(cnts);//9000,��Ƶ��1KƵ�ʼ���

}
static UINT_8 sysTickUptime_OVER =0 ;//�������������־λ
UINT_64  SYSTIME_CONF::GetSysTime_us(void) 
{
	register uint32_t ms;
	UINT_64 value;
	ms = sysTickUptime;
	if(sysTickUptime>=0xffffffff)//1193��Сʱ��Ż����
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
	Cycle_T[item][OLD] = Cycle_T[item][NOW];	//��һ�ε�ʱ��
	Cycle_T[item][NOW] = GetSysTime_us()/1000000.0f; //���ε�ʱ��
	Cycle_T[item][NEW] = ( ( Cycle_T[item][NOW] - Cycle_T[item][OLD] ) );//�����ʱ�䣨���ڣ�
	return Cycle_T[item][NEW];
}

void  SYSTIME_CONF::TIM6_Configuration(void)
{
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
		
		// ����TIMx_CLK,x[6,7],���ڲ�ʱ��CK_INT=72M
    RCC_APB1PeriphClockCmd (RCC_APB1Periph_TIM6, ENABLE);
	
		// �Զ���װ�ؼĴ����ܵ�ֵ(����ֵ)
    TIM_TimeBaseStructure.TIM_Period=1000;
	
    // �ۼ� TIM_Period��Ƶ�ʺ����һ�����»����ж�
	  // ʱ��Ԥ��Ƶ��Ϊ71����������������ʱ��CK_CNT = CK_INT / (71+1)=1M
    TIM_TimeBaseStructure.TIM_Prescaler= 71;
	
		// ʱ�ӷ�Ƶ���� ��������ʱ��TIM6��TIM7û�У����ù�
    //TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1;
		
		// ����������ģʽ��������ʱ��TIM6��TIM7ֻ�����ϼ�����û�м���ģʽ�����ã����ù�
    //TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up; 
		
		// �ظ���������ֵ��������ʱ��TIM6��TIM7û�У����ù�
		//TIM_TimeBaseStructure.TIM_RepetitionCounter=0;
	
	  // ��ʼ����ʱ��TIMx, x[6,7]
    TIM_TimeBaseInit(TIM6, &TIM_TimeBaseStructure);
		
		// ����������жϱ�־λ
    TIM_ClearFlag(TIM6, TIM_FLAG_Update);
	  
		// �����������ж�
    TIM_ITConfig(TIM6,TIM_IT_Update,ENABLE);
		
		// ʹ�ܼ�����
    TIM_Cmd(TIM6, ENABLE);																		
  

    NVIC_InitTypeDef NVIC_InitStructure; 
		
		// �����ж���Դ
    NVIC_InitStructure.NVIC_IRQChannel = TIM6_IRQn;
	
		// ���������ȼ�Ϊ 0
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	 
	  // ������ռ���ȼ�Ϊ0//���ڳ�����
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;	
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
	
}
void SYSTIME_CONF::Duty_Loop(void)   					//�����������Ϊ1ms���ܵĴ���ִ��ʱ����ҪС��1ms��
{

	if( loop.check_flag == 1 )
	{
			Duty_1ms();							//����1ms������
		
		if( loop.cnt_2ms >= 2 )
		{
			loop.cnt_2ms = 0;
			Duty_2ms();						//����2ms������
		}
		if( loop.cnt_5ms >= 5 )
		{
			loop.cnt_5ms = 0;
			Duty_5ms();						//����5ms������
		}
		if( loop.cnt_10ms >= 10 )
		{
			loop.cnt_10ms = 0;
			Duty_10ms();					//����10ms������
		}
		if( loop.cnt_20ms >= 20 )
		{
			loop.cnt_20ms = 0;
			Duty_20ms();					//����20ms������
		}
		if( loop.cnt_50ms >= 50 )
		{
			loop.cnt_50ms = 0;
			Duty_50ms();					//����50ms������
		}
		if( loop.cnt_100ms >= 100 )
		{
			loop.cnt_100ms = 0;
			Duty_100ms();					//����1000ms������
		}
		if( loop.cnt_500ms >= 500 )
		{
			loop.cnt_500ms = 0;
			Duty_500ms();					//����1000ms������
		}
		loop.check_flag = 0;		//ѭ��������ϱ�־
	}
}

void SYSTIME_CONF::Duty_1ms()
{
		ANO_DT_Data_Exchange();
	//myt = Sys_Time.Get_Cycle_T(1);
		// DMA������	


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
		loop.err_flag ++;     //ÿ�ۼ�һ�Σ�֤��������Ԥ��������û�����ꡣ
	}
	else
	{	
		loop.check_flag = 1;	//�ñ�־λ��ѭ�����������
	}
}