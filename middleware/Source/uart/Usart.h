#ifndef _USART_H
#define _USART_H
#include "stm32f10x.h"
#include "Common.h"
//#define APMLEN 200

#define APMLEN 100 //��APMͨ��ʱ����󳤶�,���ԸĴ�
#define ToAnalysis_Len 53 //��APM�����������ϵ����ݳ���
#define Frame_Len 33//�ӱ����ӷ���APM�����ݳ���

#define Packet_LEN 25
#define PC_LEN 60
#define PC_BUFFER_LEN (2*Packet_LEN) 
enum Flow_Step
{
	GROY_x = 1,
	GROY_y = 2,
	Sonar_Flow  = 3,
	Flow_comp_x  = 4,
	Flow_comp_y  = 5
};
enum AnalyseEnum
{
	ACC_x_Step = 1,
	ACC_y_Step = 2,
	ACC_z_Step  = 3,
	Angle_roll_Step  = 4,
	Angle_pitch_Step  = 5,
	Angle_yaw_Step  = 6,

	TranBackHeight_Step  = 7,
	Rserve_para1_Step  = 8,
	Groy_x_step = 9,
  Groy_y_step = 10,
	
	Rserve_para2_Step  = 11,
	Rserve_para3_Step  = 12,
	Rserve_para4_Step  = 13,
	Rserve_para5_Step  = 14
};
typedef struct
{
	//������ת
	float k_comp_x;
	float k_comp_y;
	
	float groy_x;
	float groy_y;
	float sonar_flow;
	float flow_comp_x;
	float flow_comp_y;
}para_flow;

typedef struct
{
	float ACC_x;
	float ACC_y;
	float ACC_z;
	float Angle_roll;
	float Angle_pitch;
	float Angle_yaw ;
	float TranBackHeight;
	
	float Rserve_para1 ;
	float Groy_x;
	float Groy_y;
	float Rserve_para2;
	float Rserve_para3 ;
	float Rserve_para4;
	float Rserve_para5;
}para_group;

typedef struct
{

	float X_offset_get;
	float Y_offset_get;
	float Dir_get;
	float e_get;
	float Dis_ElimateByIMG;//ͼ�����ľ���
	bool  is_GetRed;//���񵽺�Բ��־
	bool  is_GetBlue;//������Բ��־
	
	
	float x_last;//��������һ��x���������ֵ
	float y_last;//��������һ��y���������ֵ
	
	float v_x;//x������ٶ�
	float v_y;//y������ٶ�
	
	float filter_vx;//�ͼ��ٶȼ��ںϵ�x�ٶ�
	float filter_vy;
	
	float filter_vx_coeff;//�����˲�ϵ��
	float filter_vy_coeff;
	
	float X_filter_pos;
	
	float Y_filter_pos;
	
	float pixTodis;
}para_vision;

typedef struct{

	float P[2][2] ;
//{{ 1, 0 },
// { 0, 1 }};
	float Pdot[4];
//{ 0,0,0,0};
	float Q_v;//=0.3,
	float Q_acc;//=0.15; 
	float R_angle ,C_0 ; //0.5,0.1
	float q_bias, PCt_0, PCt_1, E, K_0, K_1, t_0, t_1;
	float v_acc;//0
	float v_err;//0
} filter_para;


//�������ö���
//��ǰ������ȷ�Ľӷ�
//PB10��PB11 ����  apm����
//PA2��PA3 ����  ������
//PA9��PA10  ���� ����
//PC10��PC11 ����  ����
enum USART_PORT_CONF
{
	APM_USART = 0,//PA9/TX   PA10/RX  USART1
	Reserve_USART = 1,//PA2/TX  PA3/RX  USART2
	
	Analysis_USART = 2,//PB10/TX  PB11/RX USART3
	Flow_USART = 3,//PC10/TX  PC11/RX UART4
	
	APM_USART_Baud = 115200,
	Flow_USART_Baud = 115200,
	Analysis_USART_Baud = 115200,
	Reserve_USART_Baud = 115200,

/****************
/*�ж�˵��	
/****************/
//�����ж���Ϊ2  4����ռ���ȼ���4����Ӧ���ȼ���0-3��	

		//��ռ���ȼ�0
	Ultra_USART_NVIC = 0,//��Ӧ���ȼ�0
	
	//��ռ���ȼ�1
	APM_USART_NVIC = 0,//��Ӧ���ȼ�0
	Analysis_USART_NVIC = 1,//��Ӧ���ȼ�1
	Reserve_USART_NVIC = 2,//��Ӧ���ȼ�2
	
//	//��ռ���ȼ�0
//	Ultra_USART_NVIC = 0,//��Ӧ���ȼ�0
//	//ϵͳ��ʱ���ж� ��Ӧ���ȼ�Ϊ1
//	APM_USART_32 = 2, //APM��32���ӿ���Ӧ���ȼ�Ϊ2
//	PC_USART_32 = 3, //APM��PC(����)���ӿ���Ӧ���ȼ�Ϊ3
//	
//	//��ռ���ȼ�1
//	APM_USART_32toPC_NVIC = 2,//��Ӧ���ȼ�2
//	APM_USART_PCto32_NVIC = 3,//��Ӧ���ȼ�3
//	
//	APM_USART_32toAPM_NVIC = 1,//��Ӧ���ȼ�1
//	APM_USART_APMto32_NVIC = 0,//��Ӧ���ȼ�0
//	
//	//��ռ���ȼ�2
//	//Ԥ��������ʱ�ж�
//	Reserve_USART_NVIC = 1,//��Ӧ���ȼ�1
//	Reserve_USART_DMASEND_NVIC = 0//��Ӧ���ȼ�0
	
};
class Usart
{
	public :

					u8 apmStr;
					 Usart(){}
					~Usart(){}
					int Usart_Init(int USART,unsigned int Baud);						
					void apmMAV_Send(u8 buffer);
					void Ultra_Send(u8 buffer);
					void Analysis_Send(u8 buffer);
					void Reserve_Send(unsigned char *DataToSend ,u8 data_num);
					void Float2Byte(float floatNum, unsigned char* byteArry);
					float Byte2Float(unsigned char* byteArry);
					void	generatePacket_ToAPM(unsigned char* byteArry,int len);
					
					void Target_V_Get(float pos_x_out, float pos_y_out, float dt);
					void Target_VisonData_Deal(float dt);
	private :
					 
};
ARMAPI void analysePacket(const uint8_t* str,int len);
ARMAPI void AnalysePacket_APM(const uint8_t* buffer,int len);
ARMAPI void Kalman_Filter_Init(void);
extern double KalmanFilter_vx(const double ResrcData);
extern double KalmanFilter_vy(const double ResrcData);
extern double KalmanFilter_pos_x(const double ResrcData);
extern double KalmanFilter_pos_y(const double ResrcData);
extern void Get_Flow_Prepare(UINT_8 data);
extern UINT_8 SEND2_APM[APMLEN];
extern UINT_8 GET_APM[APMLEN];
extern UINT_8 SEND2_PC[PC_LEN];
extern UINT_8 From_PC[PC_LEN];
extern para_group Para_APM;
extern para_vision Para_pos;
extern para_flow Para_Flow;
extern Usart apmMAV;
extern Usart Flow;
extern Usart AnalysisUsart;
extern Usart ReserveUsart;
extern filter_para filter_vx;
extern filter_para filter_vy;
#endif
