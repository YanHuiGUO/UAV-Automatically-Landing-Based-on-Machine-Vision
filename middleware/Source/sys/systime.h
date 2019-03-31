#ifndef _SYSTIME_H
#define _SYSTIME_H
#include "include.h"
enum
{
  //抢占优先级0
	Systime_NVIC = 3 ,//相应优先级1
};
#define CLOSE_TIME	 RCC_APB1PeriphClockCmd (RCC_APB1Periph_TIM6, DISABLE)
#define OPEN_TIME RCC_APB1PeriphClockCmd (RCC_APB1Periph_TIM6, ENABLE)

//测试运行时间暂存数组数量，这个测5处时间
#define GET_TIME_NUM 	(5)		

#define TICK_PER_SECOND 1000 
#define TICK_US	(1000000/TICK_PER_SECOND)

typedef struct
{
	UINT_8 check_flag;
	UINT_8 err_flag;
	INT_16 cnt_1ms;
	INT_16 cnt_2ms;
	INT_16 cnt_5ms;
	INT_16 cnt_10ms;
	INT_16 cnt_20ms;
	INT_16 cnt_50ms;
	INT_16 cnt_100ms;
	INT_16 cnt_500ms;
	UINT_16 time;
}loop_t;

struct TimeGroup
{
	UINT_8 SetMode_Sleep_Flag;
	UINT_32 SetMode_Sleep_Cnt;

};
class SYSTIME_CONF
{
	
	public: 
					void  SysTick_Configuration(void);
				  UINT_64  GetSysTime_us(void) ;
					FLOAT_32 Get_Cycle_T(u8 item);
					void TIM6_Configuration(void);
					void Duty_Loop(void) ;
					void Loop_check(void) ;
					void Duty_1ms(void);
					void Duty_2ms(void);
					void Duty_5ms(void);
					void Duty_10ms(void);
					void Duty_20ms(void);
					void Duty_50ms(void);
					void Duty_100ms(void); 
					void Duty_500ms(void);
	private:
					loop_t loop;
};

extern SYSTIME_CONF Sys_Time;
extern TimeGroup TimeSleep;
#endif
