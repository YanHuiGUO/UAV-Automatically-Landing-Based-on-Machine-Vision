/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
#include "MyFunc.h"
Myfun_Para mypara;
/*******************************************
�������ƣ�Float2Byte
�� �ܣ�������ת�����ֽ�������
�� ������ڲ���floatNum����ת���ĸ�����
�� �� ֵ��byteArry��ת������ֽ�����
********************************************/
void Myfun_Para::Float2Byte(float floatNum, unsigned char* byteArry)
{
	char* pchar = (char*)&floatNum;
	for (int i = 0; i<sizeof(float); i++)
	{
		*byteArry = *pchar;
		pchar++;
		byteArry++;
	}
}

/*******************************************
�������ƣ�Byte2Float
�� �ܣ��ֽ��ͣ�16���Ƹ�ʽ��ת���ɸ�����
�� ������ڲ���*byteArry��ת���ɵ��ֽ����飬ÿ�ĸ��ֽ�ת����һ�������ȸ�����
�� �� ֵ��ת����ĸ�����
********************************************/
float Myfun_Para::Byte2Float(unsigned char* byteArry)
{
	return *((float*)byteArry);
}

/*******************************************
�������ƣ�getDataAnalyse_Board
�� �ܣ���ȡ����������
�� ������ڲ��� *buffer ���յ����� ��len �������ݵ�ʵ�ʳ���
�� �� ֵ��None
********************************************/
void Myfun_Para::getDataAnalyse_Board(unsigned char  *buffer, int len)
{
	bool gethead = false;
	int recieve_start = 0;

	if (len <= BUFFER_ARR_LEN)
	{
		for (int i = 0; i < len; i++)
		{
			if (*(buffer + i) == 0xfa)
			{
				gethead = true;
				recieve_start = i;
				break;
			}
		}
		if (recieve_start + Frame_Len < BUFFER_ARR_LEN)
		{
			//cliSerial->printf_P(PSTR("PERF: %x\n"), buffer[recieve_start + 3]);
			//Э�� :֡ͷ  ��ַ ����λ    ֡��   ����������  Xƫ��  Yƫ��  ƫ��   ����߶�   У��λ
			//      0xFA  0x01  0x01   0x19(25)  4 Bytes    ...   ....   ....    ....    ͷ�ĸ��ֽ�����
			if (gethead == true
				&& buffer[recieve_start + 1] == 0x01
				&& buffer[recieve_start + 2] == 0x01
				&& buffer[recieve_start + 3] == Frame_Len
				&& buffer[recieve_start + Frame_Len - 1]
				== ((buffer[recieve_start] & buffer[recieve_start + 1] & buffer[recieve_start + 2]
				& buffer[recieve_start + 3]) + 1)
				)//ע��ƥ��Э��ĳ���ʱ����ֻ����Frame_Len����ΪAVR�ĵײ�ûDMA����32�����ڴ��������е�����
			{
				AnalyseEnum step = Sonar_Info;
				gethead = false;

				//Ҫ�ų�У��λ
				for (int i = recieve_start + 4; i < recieve_start + Frame_Len - 1 - 1; i = i + 4)
				{
					unsigned char tempdata[4] = { 0 };


					for (int j = 0; j < 4; j++)
					{
						if (i + j<BUFFER_ARR_LEN)
							tempdata[j] = buffer[i + j];
					}
					switch (step)
					{
					case Sonar_Info:mypara.Sonar_height = Byte2Float(tempdata);
						sonar_alt = mypara.Sonar_height;

						//�ڸ߶Ȼ������flag
						//cliSerial->printf_P(PSTR("%c %c %c %c\n"), tempdata[0], tempdata[1], tempdata[2], tempdata[3]);
						//cliSerial->printf_P(PSTR("%c %c %c %c\n"), tempdata[0], tempdata[1], tempdata[2], tempdata[3]);
						if (mypara.Sonar_height>0 && mypara.Sonar_height<400) //0-400CMΪ������Χ
							mypara.GetSonar_Height_Flag = true;
						else mypara.GetSonar_Height_Flag = false;
						//if (mypara.GetSonar_Height_Flag == false)
						//	cliSerial->printf_P(PSTR("DIS: 0\n"));
						//cliSerial->printf_P(PSTR("%c%c%c\n"), 'a','b','c');
						step = Target_X;
						break;
					case Target_X:	mypara.Board_data.x = Byte2Float(tempdata);
						//cliSerial->printf_P(PSTR("%f\n"), mypara.Board_data.x);
						step = Target_Y;
						break;
					case Target_Y:	mypara.Board_data.y = Byte2Float(tempdata);

						step = Target_Dir;
						break;
					case Target_Dir:mypara.Board_data.direct_angle = Byte2Float(tempdata);
						//cliSerial->printf_P(PSTR("%f\n"), mypara.Board_data.direct_angle);
						step = Target_EstiHeight;
						break;
					case Target_EstiHeight: mypara.Board_data.estimate_height = Byte2Float(tempdata);
						step = Target_V_X;
						break;
					case Target_V_X:mypara.Board_data.v_x = Byte2Float(tempdata);
						//cliSerial->printf_P(PSTR("%f\n"), mypara.Board_data.direct_angle);
						step = Target_V_Y;
						break;
					case Target_V_Y:mypara.Board_data.v_y = Byte2Float(tempdata);
						//cliSerial->printf_P(PSTR("%f\n"), mypara.Board_data.direct_angle);

						step = Sonar_Info;
						break;
						//�����������������룬ÿ��FrameLen����4��Bytes
					}
				}
			}
		}
	}
}

#ifdef USERHOOK_INIT
void userhook_init()
{
	// put your initialisation code here
	// this will be called once at start-up
	hal.uartC->begin(115200, UartBuffer_Len, UartBuffer_Len);
	mypara.Target_Pos_Mode_Flag = false;
}	
#endif
void Float2Byte(float floatNum, unsigned char* byteArry)
{
	char* pchar = (char*)&floatNum;
	for (int i = 0; i<sizeof(float); i++)
	{
		*byteArry = *pchar;
		pchar++;
		byteArry++;
	}
}
#define GeneratePacket_ToAnalysis_Len 60
static   char ToAnalysis_Arr[GeneratePacket_ToAnalysis_Len];
int PosControl_Flag = 0;
int GeneratePacket_ToAnalysis(char* byteArry, int len)
{
	unsigned char Acc_x[4] = { 0 };
	unsigned char Acc_y[4] = { 0 };
	unsigned char Acc_z[4] = { 0 };
	unsigned char Angle_roll[4] = { 0 };
	unsigned char Angle_pitch[4] = { 0 };
	unsigned char Angle_yaw[4] = { 0 };
	unsigned char TranBackHeight[4] = { 0 }; // �ش����ݣ�������Կ����Σ����糬�����ĸ߶ȿ���������,����ش�������ѹ�Ƶĸ߶�
	unsigned char Rserve_para1[4] = { 0, 0, 0, 0 };//Ԥ�������ݻش���
	unsigned char Rserve_para2[4] = { 0, 0, 0, 0 };
	unsigned char Rserve_para3[4] = { 0, 0, 0, 0 };
	unsigned char Groy_xpara[4] = { 0, 0, 0, 0 };
	unsigned char Groy_ypara[4] = { 0, 0, 0, 0 };
	//unsigned char Rserve_para4[4] = { 0, 0, 0, 0 };
	//unsigned char Rserve_para5[4] = { 0, 0, 0, 0 };
	int index = 0;

	if (len <= GeneratePacket_ToAnalysis_Len)
	{

		//����������ϵ�µļ��ٶ�ת������������ϵ��
		//��������ϵ��ͼ������ϵһ�£�ת����ӼƵ�����ϵΪ:
		//					|x
		//					|
		//				 ------>y  ��������ϵ
		//
		mypara.acc_x_body = ahrs.get_accel_ef().x*ahrs.cos_yaw() + ahrs.get_accel_ef().y*ahrs.sin_yaw();
		mypara.acc_y_body = -ahrs.get_accel_ef().x*ahrs.sin_yaw() + ahrs.get_accel_ef().y*ahrs.cos_yaw();
		Float2Byte(mypara.acc_x_body, Acc_x);
		Float2Byte(mypara.acc_y_body, Acc_y);

		Float2Byte(ahrs.get_accel_ef().z, Acc_z);


		Float2Byte(ahrs.roll, Angle_roll);
		Float2Byte(ahrs.pitch, Angle_pitch);
		Float2Byte(ahrs.yaw, Angle_yaw);

		Float2Byte(mypara.Sonar_height, TranBackHeight);

		Float2Byte(mypara.Target_Pos_Mode_Flag, Rserve_para1);
		Float2Byte(g.rc_1.control_in, Groy_xpara);
		Float2Byte(g.rc_2.control_in, Groy_ypara);
		Float2Byte(mypara.pid.out_x, Rserve_para2);
		Float2Byte(mypara.pid.out_y, Rserve_para3);

		byteArry[index] = 0xFA; index++;
		byteArry[index] = 0x01; index++;
		byteArry[index] = 0x01; index++;
		index++;
		int i = 0;
		for (i = index; i<index + 4; i++)
			byteArry[i] = Acc_x[i - index];
		index = i;

		for (i = index; i<index + 4; i++)
			byteArry[i] = Acc_y[i - index];
		index = i;

		for (i = index; i<index + 4; i++)
			byteArry[i] = Acc_z[i - index];
		index = i;

		for (i = index; i<index + 4; i++)
			byteArry[i] = Angle_roll[i - index];
		index = i;

		for (i = index; i<index + 4; i++)
			byteArry[i] = Angle_pitch[i - index];
		index = i;

		for (i = index; i<index + 4; i++)
			byteArry[i] = Angle_yaw[i - index];
		index = i;

		for (i = index; i<index + 4; i++)
			byteArry[i] = TranBackHeight[i - index];
		index = i;

		for (i = index; i<index + 4; i++)
			byteArry[i] = Rserve_para1[i - index];
		index = i;

		for (i = index; i<index + 4; i++)
			byteArry[i] = Groy_xpara[i - index];
		index = i;

		for (i = index; i<index + 4; i++)
			byteArry[i] = Groy_ypara[i - index];
		index = i;
		for (i = index; i<index + 4; i++)
			byteArry[i] = Rserve_para2[i - index];
		index = i;

		for (i = index; i<index + 4; i++)
			byteArry[i] = Rserve_para3[i - index];
		index = i;

		byteArry[index] = (byteArry[4] & byteArry[5] & byteArry[6] & byteArry[7]) + 1;

		byteArry[3] = index + 1;//index�Ǵ�0��ʼ��
	}
	return byteArry[3];
}
#ifdef USERHOOK_FASTLOOP
unsigned char AnalyseBoard[BUFFER_ARR_LEN];
void userhook_FastLoop()
{
	//���ڴ�����
	//hal.uartC->print('a');
	
	int tempcount = hal.uartC->available();
	if (tempcount > 0 && tempcount<BUFFER_ARR_LEN)
	{

		for (int i = 0; i < tempcount; i++)
		{
			AnalyseBoard[i] = (unsigned char)hal.uartC->read();

			RX_Flag = 1;
		}
		if (RX_Flag == 1)
		{

			mypara.getDataAnalyse_Board(AnalyseBoard, tempcount);
			
			//if (mypara.Sonar_height==0)cliSerial->printf_P(PSTR("GET0\n"));
			for (int j = 0; j < BUFFER_ARR_LEN; j++)
				AnalyseBoard[j] = 0;
			RX_Flag = 0;
		}
	}
	//������������������
	//int tanslen = GeneratePacket_ToAnalysis(ToAnalysis_Arr, GeneratePacket_ToAnalysis_Len);
	//for (int i = 0; i < tanslen; i++)
	//	hal.uartC->print(ToAnalysis_Arr[i]);
}
#endif

#ifdef USERHOOK_50HZLOOP
void userhook_50Hz()
{
    // put your 50Hz code here
}
#endif




//�ظ�1�򲨶��˿��أ��ظ�0��λ�˿���
int Cheak_Rc7(int16_t rc)
{
	uint8_t NowMode = -1;

	NowMode = read_3pos_switch(rc);

	if (NowMode == AUX_SWITCH_LOW) { mypara.Target_Pos_Mode_Flag = false; return  0; }
	else if (NowMode == AUX_SWITCH_HIGH) { mypara.Target_Pos_Mode_Flag = true; return 1; }
	else { mypara.Target_Pos_Mode_Flag = false; return -1; }
}
bool Myfun_Para::Target_Pos_init(bool ignore_checks)
{
		//���������ʼ��Ϊ��ֵ
		mypara.Throttle = THROTTLE_IN_MIDDLE;

		pos_control.set_speed_z(-g.pilot_velocity_z_max, g.pilot_velocity_z_max);
		pos_control.set_accel_z(g.pilot_accel_z);

		// initialise altitude target to stopping point
		pos_control.set_target_to_stopping_point_z();

		mypara.pid.out_x = mypara.pid.out_y = 0;

		mypara.pid.p_out_x = 0;
		mypara.pid.i_out_x = 0;
		mypara.pid.d_out_x = 0;

		mypara.pid.p_out_y = 0;
		mypara.pid.i_out_y = 0;
		mypara.pid.d_out_y = 0;

		return true;

}
void Myfun_Para::Set_Height_Sonar(float alt_cm)
{
	mypara.DesireHeight = alt_cm;//100CM
}
float Myfun_Para::Set_Height_Target(float alt_cm)
{
	float alt_change = alt_cm - mypara.Sonar_height*cos(ahrs.pitch)*cos(ahrs.roll);
	alt_change = constrain_float(alt_change, -35, 35);
	return alt_change;
}
int16_t Myfun_Para::Height_Target_Control()
{
	AP_Motors& motors_temp(motors);
	//�����ֹң��ҡ�˸���ʱ����Ӧ�����Լ��Ĵ���
	motors_temp.limit.throttle_lower = false;
	motors_temp.limit.throttle_upper = false;
	int16_t HeightDesire_Throttle = 500;
	int Sonar_dis = (int)(mypara.Sonar_height*cos(ahrs.pitch)*cos(ahrs.roll));
	float Height_Targe_error = 0;
	if (mypara.GetSonar_Height_Flag == true)
	{  
		if (Sonar_dis < 200)
		{

			float Height_P = 2;

			Height_Targe_error = Set_Height_Target(mypara.DesireHeight);

			int16_t Height_Now_Error = mypara.DesireHeight - Sonar_dis;

			int16_t Height_P_out = Height_P * (Height_Targe_error); //�Ŵ��ҡ���ٶ�

			Height_P_out = constrain_int16(Height_P_out, -100, 100);

			//20CM���������
			if (Height_Now_Error<-10 || Height_Now_Error>10)
			{
				if (Height_Targe_error > 0)
					HeightDesire_Throttle = THROTTLE_IN_DEADBAND_TOP + (Height_P_out);
				else if (Height_Targe_error < 0)//ע������Height_P_out����������Ǹ���
					HeightDesire_Throttle = THROTTLE_IN_DEADBAND_BOTTOM + (Height_P_out);
			}
			else HeightDesire_Throttle = THROTTLE_IN_MIDDLE;

		}
		else HeightDesire_Throttle = THROTTLE_IN_MIDDLE;
	}
	else HeightDesire_Throttle = THROTTLE_IN_MIDDLE;

	//cliSerial->printf_P(PSTR("%f,%d\n"), mypara.Sonar_height, HeightDesire_Throttle);
		/*if (mypara.Sonar_height>10)
		cliSerial->printf_P(PSTR("DIS: %f,%d \n"), mypara.Sonar_height, HeightDesire_Throttle);*/

	return HeightDesire_Throttle;
}

void Myfun_Para::Target_Pos_Control(void)
{
	if (mypara.Board_data.x == 0 && mypara.Board_data.y == 0)
	{
		//mypara.pid.out_x = mypara.pid.out_y = 0;
		return;
	}

	float center_x = 120;
	float center_y = 140;


	int16_t new_x = 0;
	int16_t new_y = 0;

	float i_x = 0;
	float i_y = 0;

	//D��Ҫ�ںϽ��ٶȣ���Ȼ�Ļ�����������ס,����D��Ҫ�󣬰ѵ���������ֵ�͸ı�߶ȵĲ��ָĵ�20HZ����
	//ע���ںϵĽ��ٶȵĴ�С�ͷ��� ahrs.get_gyro(0/1/2);
	//װ�����һ��k_camera������Ч�����

	// 3   3     0.8    0.2 ������ÿ���޷� �ǶȻ�P = 4.8����
	//3   3     0.875    0.125 ������ÿ���޷� �ǶȻ�P = 4.45����
	//������ 3 3 0.875 0.125 �ǶȻ�P =4.35 ��С�ǶȻ���P����ʹ������С��
	// 3 3 0.85 0.15 �ǶȻ�P =4.4 ��С�ǶȻ���P��������λ�û�D����ʹ������С��
	float kp_min = 1.75;
	float kp_max = 3.25;
	
	float kd_min = 15;
	float kd_max = 60;

	float err_max_x = 160;
	float err_max_y = 120;

	mypara.pid.kp_x = kp_max;
	mypara.pid.ki_x = 0;
	mypara.pid.kd_x = kd_max;

	mypara.pid.kp_y = kp_max;
	mypara.pid.ki_y = 0;
	mypara.pid.kd_y = kd_max;

	//���������Ұ�Ǿ�����ϵ��
	float k_camera  = 0.985f;
	float camera_view = 65.0f;

	float pitch_comp = constrain_float(ahrs.pitch, -0.349, 0.349);
	float roll_comp = constrain_float(ahrs.roll, -0.349, 0.349);

	//ע�⣬mypara�����Pitch��Roll�ǽǶȵģ�ahrs������ǻ��ȵ�
	mypara.Board_data.x = ahrs.pitch >= 0 ? mypara.Board_data.x - k_camera*fabs(pitch_comp) / camera_view * 240
		: mypara.Board_data.x + k_camera*fabs(pitch_comp) / camera_view * 240;

	float  x_error = -(center_x - mypara.Board_data.x);

	x_error = constrain_float(x_error, -err_max_x, err_max_x);

	////����PD������ģ��PID����������ϣ���Error�仯�еȵ�ʱ�����P��СD���
	//mypara.pid.kp_x = kp_min + (kp_max - kp_min) / (err_max_x*err_max_x / 4) \
	//				* (fabs(x_error) - err_max_x / 2)*(fabs(x_error) - err_max_x / 2);
	//mypara.pid.kd_x = kd_max - (kd_max - kd_min) / (err_max_x*err_max_x / 4) \
	//				* (fabs(x_error) - err_max_x / 2)*(fabs(x_error) - err_max_x / 2);
	////x pd�޷�
	//mypara.pid.kp_x = constrain_float(mypara.pid.kp_x, kp_min, kp_max);
	//mypara.pid.kd_x = constrain_float(mypara.pid.kd_x, kd_min, kd_max);

	mypara.Board_data.y = ahrs.roll >= 0 ? mypara.Board_data.y + k_camera*fabs(roll_comp) / camera_view * 320
		: mypara.Board_data.y - k_camera*fabs(roll_comp) / camera_view * 320;

	float  y_error = center_y - mypara.Board_data.y;
	y_error = constrain_float(y_error, -err_max_y, err_max_y);

	////y�����PD
	//mypara.pid.kp_y = kp_min + (kp_max - kp_min) / (err_max_y*err_max_y/4) \
	//				* (fabs(y_error) - err_max_y / 2)*(fabs(y_error) - err_max_y / 2);
	//mypara.pid.kd_y = kd_max - (kd_max - kd_min) / (err_max_y*err_max_y / 4) \
	//				* (fabs(y_error) - err_max_y / 2)*(fabs(y_error) - err_max_y / 2);

	////y pd�޷�
	//mypara.pid.kp_y = constrain_float(mypara.pid.kp_y, kp_min, kp_max);
	//mypara.pid.kd_y = constrain_float(mypara.pid.kd_y, kd_min, kd_max);

	//x��PID���
	mypara.pid.p_out_x = mypara.pid.kp_x * x_error;
	i_x = mypara.pid.ki_x* x_error;
	mypara.pid.i_out_x += (int16_t)i_x;
	mypara.pid.i_out_x = constrain_int16(mypara.pid.i_out_x, -500,500);

	mypara.pid.d_out_x = (int16_t)(mypara.pid.kd_x*(0.8*(x_error - mypara.pid.d_last_x) + 0.2*ahrs.get_gyro().y));

	mypara.pid.out_x = ((int16_t)mypara.pid.p_out_x + mypara.pid.i_out_x + mypara.pid.d_out_x);

	//mypara.pid.out_x = constrain_int16(new_x, mypara.pid.out_x - 200, mypara.pid.out_x + 200);

	mypara.pid.out_x = constrain_int16(mypara.pid.out_x, -1000, 1000);

	mypara.pid.out_x = mypara.pid.out_x;


	//y��PID���
	mypara.pid.p_out_y = mypara.pid.kp_y * y_error;
	i_y = mypara.pid.ki_y* y_error;
	mypara.pid.i_out_y += (int16_t)i_y;

	mypara.pid.i_out_y = constrain_int16(mypara.pid.i_out_y, -500, 500);

	mypara.pid.d_out_y = (int16_t)(mypara.pid.kd_y*(0.8*(y_error - mypara.pid.d_last_y) + 0.2*(-ahrs.get_gyro().x)));

	mypara.pid.out_y = ((int16_t)mypara.pid.p_out_y + mypara.pid.i_out_y + mypara.pid.d_out_y);

	//mypara.pid.out_y = constrain_int16(new_y, mypara.pid.out_y - 200, mypara.pid.out_y + 200);

	mypara.pid.out_y = constrain_int16(mypara.pid.out_y, -1000, 1000);

	mypara.pid.out_y = mypara.pid.out_y;

	mypara.pid.d_last_x = x_error;
	mypara.pid.d_last_y = y_error;


}


void Myfun_Para::Target_Pos_run(void)
{
	int16_t target_roll, target_pitch;
	float target_yaw_rate;
	int16_t target_climb_rate;
	//cliSerial->printf_P(PSTR("DIS: %d,%d,%d,%d\n"), g.rc_1.control_in, g.rc_2.control_in, g.rc_3.control_in, g.rc_4.control_in;)	
	
	//����ͼ������POS�������ط�
	get_pilot_desired_lean_angles(mypara.pid.out_y, mypara.pid.out_x, target_roll, target_pitch);
	
	// get pilot's desired yaw rate
	target_yaw_rate = get_pilot_desired_yaw_rate(g.rc_4.control_in);

	// get pilot desired climb rate
	//���ó������������ط�
	//����500����ֵ������400-600 ��������600���ϣ��½���400����

	//��ʹ�ó������������ʱ��������������ĳ� mypara.Throttle����
	target_climb_rate = get_pilot_desired_climb_rate(g.rc_3.control_in);

	attitude_control.angle_ef_roll_pitch_rate_ef_yaw(target_roll, target_pitch, target_yaw_rate);
		
	//set_alt_target_with_slew(cm,dt) ����ֱ�����ø߶�
	 
	pos_control.set_alt_target_from_climb_rate(target_climb_rate, G_Dt);

	pos_control.update_z_controller();
	

}
#ifdef USERHOOK_MEDIUMLOOP
void userhook_MediumLoop()
{
	// put your 20Hz code here
	
	if (Cheak_Rc7(g.rc_7.radio_in) == 1)
	{
		//set_mode(Target_Pos);
	}


	if (mypara.Target_Pos_Mode_Flag == true)
	{
		//��	_pos_target.z���Բ����������߶������� pos_to_rate_z
		mypara.Set_Height_Sonar(100); //����100cm
		mypara.Throttle = mypara.Height_Target_Control();
		mypara.Target_Pos_Control();


		mypara.yaw = ahrs.yaw * 180 / PI;
		mypara.pitch = ahrs.pitch * 180 / PI;
		mypara.roll = ahrs.roll * 180 / PI;

		cliSerial->printf_P(PSTR("DIS: %d"),1);
		/*Log_Write_Pos();*/
	}
	
	else
		return;

	
	//Vector3f bf_vector, angle_ef_error, _angle_ef_target;
	//_angle_ef_target.x = 0;
	//angle_ef_error.x = wrap_180_cd_float(_angle_ef_target.x - ahrs.roll_sensor);

	//_angle_ef_target.y = 0;
	//angle_ef_error.y = wrap_180_cd_float(_angle_ef_target.y - ahrs.pitch_sensor);

	//angle_ef_error.z = wrap_180_cd(_angle_ef_target.z - ahrs.yaw_sensor);

	//// update yaw angle target to be within max angle overshoot of our current heading
	//_angle_ef_target.z = angle_ef_error.z + ahrs.yaw_sensor;


	//bf_vector.x = angle_ef_error.x - ahrs.sin_pitch() * angle_ef_error.z;
	//bf_vector.y = ahrs.cos_roll()  * angle_ef_error.y + ahrs.sin_roll() * ahrs.cos_roll() * angle_ef_error.z;
	//bf_vector.z = -ahrs.sin_roll() * angle_ef_error.y + ahrs.cos_pitch() * ahrs.cos_roll() * angle_ef_error.z;
	/*cliSerial->printf_P(PSTR("PERF: %lf %lf %lf *** %lf *** %lf %lf \n"),
		mypara.yaw,
		mypara.pitch,
		mypara.roll,
		ahrs.cos_roll(),bf_vector.x,bf_vector.y);*/

}
#endif

#ifdef USERHOOK_SLOWLOOP
void userhook_SlowLoop()
{

    // put your 3.3Hz code here
}
#endif

#ifdef USERHOOK_SUPERSLOWLOOP
bool log_start = false;
void userhook_SuperSlowLoop()
{

	/*float k_camera = 0.9f;
	mypara.pitch = ahrs.pitch * 180 / PI;
	mypara.roll = ahrs.roll * 180 / PI;
	float pitch_comp = constrain_float(ahrs.pitch, -0.349, 0.349);
	float roll_comp = constrain_float(ahrs.roll, -0.349, 0.349);
	float x_income = mypara.Board_data.x;
	float y_income = mypara.Board_data.y;
	mypara.Board_data.x = ahrs.pitch >= 0 ? mypara.Board_data.x - k_camera*tan(fabs(pitch_comp)) * 240
		: mypara.Board_data.x + k_camera*tan(fabs(pitch_comp)) * 240;
	mypara.Board_data.y = ahrs.roll >= 0 ? mypara.Board_data.y + k_camera*tan(fabs(roll_comp)) * 320
		: mypara.Board_data.y - k_camera*tan(fabs(roll_comp)) * 320;
	cliSerial->printf_P(PSTR("PERF:%f,%f,%f,%f,%f,%f \n"), ahrs.get_gyro().x, ahrs.get_gyro().y, mypara.pitch, mypara.roll, mypara.Board_data.x, mypara.Board_data.y);*/
    // put your 1Hz code here
	//cliSerial->printf_P(PSTR("%f\n"), mypara.Sonar_height);
}
#endif