#ifndef _DA_ULTRA_H
#define _DA_ULTRA_H
#include "include.h"
enum UltraType
{
	US100 = 0,
	KS103 = 1,
	HR_04 = 2,
	
	//超声波串口缓冲数组长度
	UltraBufferLen = 5
};
class UltraConf
{
	public :
						 UltraConf();
						void DAC_CH1_INIT(void);
						int Ultra_INIT(int);
						UINT_16 GetUltra(const UINT_8 buffer);
						void Dac1_Set_Vol(void);
						void Ultra_TX(int UltraType);
	
						//超声波数据OK标志位
						UINT_8 Ultra_Ok;
	private :
						UINT_8 Ultra_TX_Arr[UltraBufferLen];
						UINT_8 Ultra_RX_Arr[UltraBufferLen];
						UINT_8 ultra_cnt;
						float UltraHeight;
};
extern float Moving_Average(UINT_8 item,UINT_8 width_num,float in);
extern UltraConf Soner;
#endif
