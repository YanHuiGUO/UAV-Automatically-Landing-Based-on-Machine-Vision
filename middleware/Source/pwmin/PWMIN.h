#ifndef _PWMIN_H
#define _PWMIN_H
#include "Common.h"
enum 
{
	//��ռ���ȼ�2
	PWMIN_NVIC = 0,//��Ӧ���ȼ�0
};
class PWMIN_CONF
{
	public:
					PWMIN_CONF(){
												TIM4CH1_CAPTURE_STA =0;	//ͨ��1���벶���־������λ�������־����6λ�������־		
												TIM4CH1_CAPTURE_UPVAL=0;
												TIM4CH1_CAPTURE_DOWNVAL=0;

												TIM4CH2_CAPTURE_STA =0;	//ͨ��2���벶���־������λ�������־����6λ�������־		
												TIM4CH2_CAPTURE_UPVAL=0;
												TIM4CH2_CAPTURE_DOWNVAL=0;

												TIM4CH3_CAPTURE_STA=0 ;	//ͨ��3���벶���־������λ�������־����6λ�������־		
												TIM4CH3_CAPTURE_UPVAL=0;
												TIM4CH3_CAPTURE_DOWNVAL=0;

												TIM4CH4_CAPTURE_STA=0 ;	//ͨ��4���벶���־������λ�������־����6λ�������־		
												TIM4CH4_CAPTURE_UPVAL=0;
												TIM4CH4_CAPTURE_DOWNVAL=0;

												tim4_T1=0;
												tim4_T2=0;
												tim4_T3=0;
												tim4_T4=0;
											}
					int PWMIN_INIT(UINT_16 arr,UINT_16 psc);
					UINT_32 pwmWidthCH1 ;	//�����ܸߵ�ƽ��ʱ��
					UINT_32 pwmWidthCH2 ;	//�����ܸߵ�ƽ��ʱ��
					UINT_32 pwmWidthCH3 ;	//�����ܸߵ�ƽ��ʱ��
					UINT_32 pwmWidthCH4 ;	//�����ܸߵ�ƽ��ʱ��
					void PWMIN_Intterupt(void);
	private:
					 u8 TIM4CH1_CAPTURE_STA ;	//ͨ��1���벶���־������λ�������־����6λ�������־		
					 u16 TIM4CH1_CAPTURE_UPVAL;
					 u16 TIM4CH1_CAPTURE_DOWNVAL;

					 u8 TIM4CH2_CAPTURE_STA ;	//ͨ��2���벶���־������λ�������־����6λ�������־		
					 u16 TIM4CH2_CAPTURE_UPVAL;
					 u16 TIM4CH2_CAPTURE_DOWNVAL;

					 u8 TIM4CH3_CAPTURE_STA ;	//ͨ��3���벶���־������λ�������־����6λ�������־		
					 u16 TIM4CH3_CAPTURE_UPVAL;
					 u16 TIM4CH3_CAPTURE_DOWNVAL;

					 u8 TIM4CH4_CAPTURE_STA ;	//ͨ��4���벶���־������λ�������־����6λ�������־		
					 u16 TIM4CH4_CAPTURE_UPVAL;
					 u16 TIM4CH4_CAPTURE_DOWNVAL;

					 u32 tim4_T1;
					 u32 tim4_T2;
					 u32 tim4_T3;
					 u32 tim4_T4;
};
extern PWMIN_CONF  PWMIN;
#endif
