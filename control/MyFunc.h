#ifndef _MyFunc_H
#define _MyFunc_H

#include <math.h>
#include <stdio.h>
#include <stdarg.h>
#include <AP_HAL.h>
#include <AP_HAL_AVR.h>
#include <AP_HAL_AVR_SITL.h>
#include <AP_HAL_PX4.h>
#include <AP_HAL_VRBRAIN.h>
#include <AP_HAL_FLYMAPLE.h>
#include <AP_HAL_Linux.h>
#include <AP_HAL_Empty.h>
#include <AP_HAL.h>
#include <AC_PosControl.h>
#include <avr/interrupt.h>
#include <avr/io.h>
#include "utility/pins_arduino_mega.h"
#include "GPIO.h"
#include "AP_MOTORS_Class.h"

#define USERHOOK_SUPERSLOWLOOP

#define USERHOOK_MEDIUMLOOP
#define USERHOOK_FASTLOOP
#define USERHOOK_INIT
#define Frame_Len 33

//设定缓冲区大小与解析板要相同
//使每次接收缓冲区内比存在一帧完整的数据
#define UartBuffer_Len (2*Frame_Len)
#define BUFFER_ARR_LEN (2*UartBuffer_Len)
enum AnalyseEnum
{
	Sonar_Info = 1,
	Target_X = 2,
	Target_Y = 3,
	Target_Dir = 4,
	Target_EstiHeight = 5,
	Target_V_X = 6,
	Target_V_Y = 7
};

class target_info
{
public:
	float x;//x向偏移
	float y;//y向偏移
	float direct_angle;//目标角度
	float estimate_height;//根据像素估算的高度

	float x_last;//记录X方向上一次的像素
	float y_last;//记录Y方向上一次的像素
	float v_x;//根据像素解算出x方向的速度
	float v_y;//根据像素解算出y方向的速度


	float filter_vx;//结合加速度计互补滤波后的vx
	float filter_vy;//滤波后的vy
	float filter_vx_coeff;
	float filter_vy_coeff;

	float pixTodis;//像素转换成距离的系数
};

class Myfunpid
{
public:
	float kp_x;
	float kp_y;

	float ki_x;
	float ki_y;

	float kd_x;
	float kd_y;

	float p_out_x;
	float p_out_y;

	float i_out_x;
	float i_out_y;


	float d_last_x;
	float d_last_y;
	float d_out_x;
	float d_out_y;

	float d_last_out_x;
	float d_last_out_y;

	int16_t out_x;
	int16_t out_y;

	int16_t temp_out_x;
	int16_t temp_out_y;

};

class Myfun_Para
{
public:

	double acc_x_body;
	double acc_y_body;


	double yaw;

	double pitch;

	double roll;

	double baro_height;

	float Sonar_height;//超声波数据
	int16_t DesireHeight;
	int16_t Throttle;

	bool GetSonar_Height_Flag; // 获得声波标志

	bool GetBoard_Data_Flag;//获得靶标标志

	bool Target_Pos_Mode_Flag; // 切到靶标定点模式

	target_info Board_data;//靶标信息

	Myfunpid v_pid;
	Myfunpid pid;

	bool Target_Pos_init(bool ignore_checks);
	void Target_Pos_Control(void);
	void Target_Pos_run(void);
	float Set_Height_Target(float alt_cm);
	int16_t  Height_Target_Control(void);
	void Set_Height_Sonar(float alt_cm);
	void getDataAnalyse_Board(unsigned char  *buffer, int len);
	void Float2Byte(float floatNum, unsigned char* byteArry);
	float Byte2Float(unsigned char* byteArry);

};
extern Myfun_Para mypara;
extern void Log_Write_Pos(void);
#endif