#ifndef _WP_H
#define _WP_H
class WP_TASK_Conf
{
	public:
					enum BP_MOED
					{
						BP_MOED_STOP = 0,
						BP_MOED_Guided = 1,//ָ��ģʽ�½��жϵ�����
						BP_MOED_SETAUTO = 2//�Զ�����ģʽ�½��жϵ�����	
					};
					enum WpState_
					{
						WpState_1 = 0,
						WpState_2 ,
						WpState_3 ,
						WpState_4 ,
						WpState_5 ,
						WpState_6
					};
					enum 
					{
						GuideTaskCmdNum = 5, //����ģʽ�������ظ�ִ��5��
						WpTaskCmdNum = 10//����ģʽ�������ظ�ִ��10��
					};
					UINT_8 GetAck;
					UINT_8 GetReq;
					UINT_8 START_WP_TASK;
					
					//��¼�ش���gps����
					FLOAT_64 lat;
					FLOAT_64 lng;
					FLOAT_32 alt;
					
					WP_TASK_Conf()
					{
						GetAck  = 0;
						START_WP_TASK = 0;
						CmdCount = 0 ;
						lat = 0;
						lng = 0;
						alt = 0;
						WpState = WpState_1;
					}
					void BreakPoint_Continue(BP_MOED way);
					void BP_MOED_Guided_Task(void);
					void BP_MOED_SETAUTO_Task(void);
					void Clear_WpFlag(void);
				
	private:	
					int WpState;
					int CmdCount;//���������¼
					mavlink_set_mode_t mode;
};
extern WP_TASK_Conf WP_TASK;
#endif

