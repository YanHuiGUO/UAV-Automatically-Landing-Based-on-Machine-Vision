#ifndef _ultrasonic_H
#define _ultrasonic_H
#include "include.h"
#define ULTRA_TRIGPORT			GPIOE
#define ULTRA_ECHOPORT			GPIOE
#define ULTRA_TRIGCLK       RCC_APB2Periph_GPIOE
#define ULTRA_ECHOCLK       RCC_APB2Periph_GPIOE
#define ULTRA_TRIG			GPIO_Pin_7
#define ULTRA_ECHO			GPIO_Pin_8

#define TRIG_Send  GPIO_SetBits(ULTRA_TRIGPORT, ULTRA_TRIG);
#define ECHO_Reci  GPIO_ReadBits(ULTRA_ECHOPORT, ULTRA_ECHO);

#define SONAR_SCALE	100.0f
#define SONAR_MIN	0.05f		/** 0.12m sonar minimum distance */
#define SONAR_MAX	3.5f		/** 3.50m sonar maximum distance */
class UltraConf
{
	public :
						float UltraHeight;
						float UltraHeight_raw;
						UltraConf():SONAR_KALMAN_L1(0.8461f),SONAR_KALMAN_L2(6.2034f){};
						void TIM2_Init(uint32_t TIM_scale, uint32_t TIM_Period);
						bool Ultran_Init(void);
						UINT_16 GetUltra(const UINT_8 buffer);
						void Ultra_TX(int UltraType);
						void sonar_read(float* sonar_value_filtered, float* sonar_value_raw);
						void sonar_filter(void);
						void Ultra_Distance(void);
						void Ultra_Ranging(void);
						UINT_8 Ultra_Ok;
	private :
						UINT_8 ultra_cnt;
						float SONAR_KALMAN_L1;
						float SONAR_KALMAN_L2;
};
extern float Moving_Average(UINT_8 item,UINT_8 width_num,float in);
extern UltraConf Sonar;
bool sonar_read(float* sonar_value_filtered, float* sonar_value_raw);
extern float sonar_distance_filtered ; // distance in meter
extern float sonar_distance_raw ; // distance in meter
extern bool distance_valid ;
uint32_t get_sonar_measure_time(void);
ARMAPI void EXTI9_5_IRQHandler(void);
extern float distance;
extern int Ultra_OK;
#endif
