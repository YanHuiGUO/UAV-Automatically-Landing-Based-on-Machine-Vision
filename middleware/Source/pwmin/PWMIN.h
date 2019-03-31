#ifndef _PWMIN_H
#define _PWMIN_H
#include "Common.h"
enum 
{
	//抢占优先级2
	PWMIN_NVIC = 0,//响应优先级0
};
class PWMIN_CONF
{
	public:
					PWMIN_CONF(){
												TIM4CH1_CAPTURE_STA =0;	//通道1输入捕获标志，高两位做捕获标志，低6位做溢出标志		
												TIM4CH1_CAPTURE_UPVAL=0;
												TIM4CH1_CAPTURE_DOWNVAL=0;

												TIM4CH2_CAPTURE_STA =0;	//通道2输入捕获标志，高两位做捕获标志，低6位做溢出标志		
												TIM4CH2_CAPTURE_UPVAL=0;
												TIM4CH2_CAPTURE_DOWNVAL=0;

												TIM4CH3_CAPTURE_STA=0 ;	//通道3输入捕获标志，高两位做捕获标志，低6位做溢出标志		
												TIM4CH3_CAPTURE_UPVAL=0;
												TIM4CH3_CAPTURE_DOWNVAL=0;

												TIM4CH4_CAPTURE_STA=0 ;	//通道4输入捕获标志，高两位做捕获标志，低6位做溢出标志		
												TIM4CH4_CAPTURE_UPVAL=0;
												TIM4CH4_CAPTURE_DOWNVAL=0;

												tim4_T1=0;
												tim4_T2=0;
												tim4_T3=0;
												tim4_T4=0;
											}
					int PWMIN_INIT(UINT_16 arr,UINT_16 psc);
					UINT_32 pwmWidthCH1 ;	//捕获总高电平的时间
					UINT_32 pwmWidthCH2 ;	//捕获总高电平的时间
					UINT_32 pwmWidthCH3 ;	//捕获总高电平的时间
					UINT_32 pwmWidthCH4 ;	//捕获总高电平的时间
					void PWMIN_Intterupt(void);
	private:
					 u8 TIM4CH1_CAPTURE_STA ;	//通道1输入捕获标志，高两位做捕获标志，低6位做溢出标志		
					 u16 TIM4CH1_CAPTURE_UPVAL;
					 u16 TIM4CH1_CAPTURE_DOWNVAL;

					 u8 TIM4CH2_CAPTURE_STA ;	//通道2输入捕获标志，高两位做捕获标志，低6位做溢出标志		
					 u16 TIM4CH2_CAPTURE_UPVAL;
					 u16 TIM4CH2_CAPTURE_DOWNVAL;

					 u8 TIM4CH3_CAPTURE_STA ;	//通道3输入捕获标志，高两位做捕获标志，低6位做溢出标志		
					 u16 TIM4CH3_CAPTURE_UPVAL;
					 u16 TIM4CH3_CAPTURE_DOWNVAL;

					 u8 TIM4CH4_CAPTURE_STA ;	//通道4输入捕获标志，高两位做捕获标志，低6位做溢出标志		
					 u16 TIM4CH4_CAPTURE_UPVAL;
					 u16 TIM4CH4_CAPTURE_DOWNVAL;

					 u32 tim4_T1;
					 u32 tim4_T2;
					 u32 tim4_T3;
					 u32 tim4_T4;
};
extern PWMIN_CONF  PWMIN;
#endif
