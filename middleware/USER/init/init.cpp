#include "include.h"
int Analysis_board_Init(void)
{
	
	
 // �����ж���Ϊ2  4����ռ���ȼ���4����Ӧ���ȼ���0-3��
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	
	//�رռ�ʱ��ʱ�ӣ��ȴ���ʼ�����
	CLOSE_TIME;
	
	//�δ�ʱ�ӳ�ʼ��
	Sys_Time.SysTick_Configuration();
	
	//ϵͳ��ʱʱ�ӳ�ʼ��
	Sys_Time.TIM6_Configuration();
	
	//��ʱʱ�ӳ�ʼ��
	while(!delay_init(72));	   
	
	//���ڳ�ʼ��
	while(!apmMAV.Usart_Init(APM_USART,APM_USART_Baud));

	while(!AnalysisUsart.Usart_Init(Analysis_USART,Analysis_USART_Baud));
	
	while(!ReserveUsart.Usart_Init(Reserve_USART,Reserve_USART_Baud));
	
	while(!Flow.Usart_Init(Flow_USART,Flow_USART_Baud));
	//��߳�ʼ��
	while(!Sonar.Ultran_Init());
	
	//�������˲�ϵ����ʼ�� ���в���ͼ�����
	Kalman_Filter_Init();
	

//	//PWM��� 50HZ��Ĭ��ռ�ձ�0��4��ͨ������
//	while(!PWMOUT.PWMOUT_INIT(50));
//	
//	//�Զ���װֵΪ0����Ƶ72����1MƵ�ʲ���
//	while(!PWMIN.PWMIN_INIT(0xffff,72-1));
	

	
	//��ʼ����ɺ�����ʱ��
	OPEN_TIME;
	
	return true;
}

