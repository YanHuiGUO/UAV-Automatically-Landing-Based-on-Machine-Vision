#include "include.h"
WP_TASK_Conf WP_TASK;
//�����õĺ���
Locationwp testwp0 ={WAYPOINT,0,0,0,0,0,30.5168783,114.3392539,0};
Locationwp testwp1 ={WAYPOINT,0,0,0,0,0,30.5169382,114.3393396,2};
Locationwp testwp2 ={LOITER_UNLIM,0,0,0,0,0,30.5170631,114.3392753,2};

//�ϵ���������
//����ѡ�����������ʵ�ַ�ʽ
void WP_TASK_Conf::BreakPoint_Continue(BP_MOED way)
{	 
	//����ģʽ
	if(way == BP_MOED_Guided)
	{

		BP_MOED_Guided_Task();
	}
	//�Զ�ģʽ
	else if(BP_MOED_SETAUTO == way )
	{
		BP_MOED_SETAUTO_Task();
	}
}	
void WP_TASK_Conf::BP_MOED_Guided_Task(void)
{
	CmdCount ++;
	mavlink_set_mode_t mode = {Guided, MAV_APM_sysid,CUSTOM_MODE_ENABLED};
	switch(WpState)
		{
			case WpState_1: mavMsgFunc.setMode(mode);
											WpState = WpState_2;
											break;
			
			case WpState_2: mavMsgFunc.setGuidedModeWP(testwp1,1);
											WpState = WpState_3;
											break;
			
			case WpState_3: 
											WpState = WpState_1;
											if(CmdCount >= GuideTaskCmdNum)
											{
												CmdCount = 0;
												//���������ֹͣ����
												Clear_WpFlag();
											}
											break;
		}
}
void WP_TASK_Conf::BP_MOED_SETAUTO_Task(void)
{
	CmdCount ++;
	static int state1count = 0;
	switch(WpState)
		{
			case WpState_1:
										mavMsgFunc.setWPTotal(3);
										state1count ++;
										//����3��
										if(state1count <=3)
											WpState = WpState_1;
										else 
										{
											state1count = 0;
											WpState = WpState_2;
										}
										break;
			case WpState_2:

										mavMsgFunc.setWP(testwp0,(ushort)0,GLOBAL_RELATIVE_ALT,0,1,false);
										WpState = WpState_3;	
										break;
			case WpState_3:
											
										mavMsgFunc.setWP(testwp1,(ushort)1,GLOBAL_RELATIVE_ALT,0,1,false);
										WpState = WpState_4;	
										break;
			case WpState_4:	
//										if(GetAck == 1) //�õ�������Ӧ�ˣ����ʹ�֡���������һ״̬
//										{
//											GetAck  = 0;
//											WpState = WpState_5;	
//										}
//										else {
//													WpState = WpState_3;
//													break;
//										}//�����ط���һ������
										mavMsgFunc.setWP(testwp2,(ushort)2,GLOBAL_RELATIVE_ALT,0,1,false);	
										WpState = WpState_5;										
										break;
			case WpState_5:	
//										if(GetAck == 1) //�õ�������Ӧ�ˣ����ʹ�֡���������һ״̬
//										{
//											GetAck  = 0;
//											WpState = WpState_1;	
//										}
//										else {
//													WpState = WpState_4;
//													break;
//										}//�����ط���һ������
										mavMsgFunc.setWPACK();
										
										if(CmdCount >= WpTaskCmdNum)
											{
												CmdCount = 0;
												//���������ֹͣ����
												Clear_WpFlag();
											}
											
										WpState = WpState_1;	
										break;	
		}
}
//��շ���ʱ���ж�ͨ������������־λ��
void WP_TASK_Conf::Clear_WpFlag(void)
{
		WP_TASK.START_WP_TASK = 0;
		WP_TASK.GetAck = 0;
		WP_TASK.GetReq = 0;
		mavMsgFlag.WpCommond.S_1 = 0;
		mavMsgFlag.WpCommond.S_2 = 0;
		mavMsgFlag.WpCommond.S_3 = 0;
}
	
	