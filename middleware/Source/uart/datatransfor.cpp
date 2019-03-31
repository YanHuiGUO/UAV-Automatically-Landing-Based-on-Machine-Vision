#include "include.h"
/////////////////////////////////////////////////////////////////////////////////////
//���ݲ�ֺ궨�壬�ڷ��ʹ���1�ֽڵ���������ʱ������int16��float�ȣ���Ҫ�����ݲ�ֳɵ����ֽڽ��з���
#define BYTE0(dwTemp)       ( *( (char *)(&dwTemp)		) )
#define BYTE1(dwTemp)       ( *( (char *)(&dwTemp) + 1) )
#define BYTE2(dwTemp)       ( *( (char *)(&dwTemp) + 2) )
#define BYTE3(dwTemp)       ( *( (char *)(&dwTemp) + 3) )

dt_flag_t f;					//��Ҫ�������ݵı�־
u8 data_to_send[50];	//�������ݻ���

/////////////////////////////////////////////////////////////////////////////////////
//Data_Exchange��������������ݷ������󣬱�����ʵ��ÿ5ms����һ�δ�������������λ�������ڴ˺�����ʵ��
//�˺���Ӧ���û�ÿ1ms����һ��
void ANO_DT_Data_Exchange(void)
{
	static u8 cnt = 0;
	static u8 senser_cnt 	= 10;
	static u8 status_cnt 	= 15;
	static u8 rcdata_cnt 	= 20;
	static u8 motopwm_cnt	= 20;
	static u8 power_cnt		=	50;
	static u8 mypara_cnt  = 10;//�Զ���Ĳ���10ms����һ��
	if((cnt % senser_cnt) == (senser_cnt-1))
		f.send_senser = 1;	
	
	if((cnt % status_cnt) == (status_cnt-1))
		f.send_status = 1;	
	
	if((cnt % rcdata_cnt) == (rcdata_cnt-1))
		f.send_rcdata = 1;	
	
	if((cnt % motopwm_cnt) == (motopwm_cnt-1))
		f.send_motopwm = 1;	
	
	if((cnt % power_cnt) == (power_cnt-1))
		f.send_power = 1;		
	if((cnt % mypara_cnt) == (mypara_cnt-1))
		f.send_mypara	= 1;
	cnt++;
/////////////////////////////////////////////////////////////////////////////////////
	if(f.send_version)
	{
		f.send_version = 0;
		ANO_DT_Send_Version(4,300,100,400,0);
	}
/////////////////////////////////////////////////////////////////////////////////////
	else if(f.send_status)
	{
		f.send_status = 0;
//		ANO_DT_Send_Status(SOFT_Roll,SOFT_Pitch,SOFT_Yaw,Wind_Pos.X_POS,Wind_Pos.Y_POS,0);
	}	
/////////////////////////////////////////////////////////////////////////////////////
	else if(f.send_senser)
	{
		f.send_senser = 0;
//		ANO_DT_Send_Senser(mpu6050.Acc.x,mpu6050.Acc.y,mpu6050.Acc.z,
//												mpu6050.Gyro.x,mpu6050.Gyro.y,mpu6050.Gyro.z,
//												0,0,0,0);
	}
/////////////////////////////////////////////////////////////////////////////////////
	else if(f.send_rcdata)
	{
		f.send_rcdata = 0;
		ANO_DT_Send_RCData(0,0,0,0,0,0,0,0,0,0);//ң�ؽ���
	}	
/////////////////////////////////////////////////////////////////////////////////////	
	else if(f.send_motopwm)
	{
		f.send_motopwm = 0;
		ANO_DT_Send_MotoPWM(0,0,0,0,0,0,0,0);//�������
	}	
/////////////////////////////////////////////////////////////////////////////////////
	else if(f.send_power)
	{
		f.send_power = 0;
//		ANO_DT_Send_Power(ultra_count,456);
	}
/////////////////////////////////////////////////////////////////////////////////////
	else if(f.send_pid1)
	{
		f.send_pid1 = 0;
//	ANO_DT_Send_PID(thr,0,0,0,0,0,0,0,0,0);
	}	
/////////////////////////////////////////////////////////////////////////////////////
	else if(f.send_pid2)
	{
		f.send_pid2 = 0;
//		ANO_DT_Send_PID(2,ctrl_1.PID[PID4].kp,ctrl_1.PID[PID4].ki,ctrl_1.PID[PID4].kd,
//											ctrl_1.PID[PID5].kp,ctrl_1.PID[PID5].ki,ctrl_1.PID[PID5].kd,
//											ctrl_1.PID[PI	D6].kp,ctrl_1.PID[PID6].ki,ctrl_1.PID[PID6].kd);
	}
/////////////////////////////////////////////////////////////////////////////////////
	else if(f.send_pid3)
	{
		f.send_pid3 = 0;
//		ANO_DT_Send_PID(3,ctrl_2.PID[PIDROLL].kp,ctrl_2.PID[PIDROLL].ki,ctrl_2.PID[PIDROLL].kd,
//											ctrl_2.PID[PIPITCH].kp,ctrl_2.PID[PIDPITCH].ki,ctrl_2.PID[PIDPITCH].kd,
//											ctrl_2.PID[PIDYAW].kp,ctrl_2.PID[PIDYAW].ki,ctrl_2.PID[PIDYAW].kd);
	}
		else if(f.send_mypara)
	{
		f.send_mypara = 0;
		
		//*****************
		//ע�ⷢ�͵���λ����ֵ���ǳ���100���ģ����������߼��Ϳ�����Ƶ�ʱ��Ҫ����ʾ��ֵ����100���м�
		//*****************
		//ANO_DT_Send_Mypara(Para_pos.X_offset_get, Para_pos.Y_offset_get ,Para_pos.Dir_get,Para_APM.ACC_x,Para_APM.ACC_y,Para_APM.Angle_pitch*180/3.141592,Para_APM.Angle_roll*180/3.141592,Para_APM.Angle_yaw*180/3.141592,Sonar.UltraHeight);
		ANO_DT_Send_Mypara(Para_pos.X_offset_get/100,Para_pos.Y_offset_get/100,Para_pos.filter_vx/100,Para_pos.filter_vy/100,Para_APM.Rserve_para2/100,Para_APM.Rserve_para3/100,Para_APM.Groy_x/100,Para_APM.Groy_y/100,Para_APM.Rserve_para1);
//		ANO_DT_Send_Mypara(Para_APM.Angle_pitch*180/3.141592,Para_APM.Angle_roll*180/3.141592,Para_APM.Rserve_para2/100,Para_APM.Rserve_para3/100,Para_pos.filter_vx,Para_pos.filter_vy,filter_vx.v_acc,filter_vy.v_acc,Para_APM.Rserve_para1);
//	ANO_DT_Send_Mypara(Para_pos.v_x,Para_pos.v_y,Para_Flow.groy_x,Para_Flow.groy_y,Para_pos.filter_vx,Para_pos.filter_vy,Para_Flow.flow_comp_x,Para_Flow.flow_comp_y,Para_APM.TranBackHeight);
	}
	
/////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////			
/////////////////////////////////////////////////////////////////////////////////////
}

/////////////////////////////////////////////////////////////////////////////////////
//Send_Data������Э�������з������ݹ���ʹ�õ��ķ��ͺ���
//��ֲʱ���û�Ӧ��������Ӧ�õ����������ʹ�õ�ͨ�ŷ�ʽ��ʵ�ִ˺���
void ANO_DT_Send_Data(u8 *dataToSend , u8 length)
{
	ReserveUsart.Reserve_Send(data_to_send, length);
}

static void ANO_DT_Send_Check(u8 head, u8 check_sum)
{	
	u8 ADD = 0;
	u8 i = 0;
	data_to_send[0]=0xAA;
	data_to_send[1]=0xAA;
	data_to_send[2]=0xEF;
	data_to_send[3]=2;
	data_to_send[4]=head;
	data_to_send[5]=check_sum;
	
	

	for(i=0;i<6;i++)
		ADD += data_to_send[i];
	data_to_send[6]=ADD;

	ANO_DT_Send_Data(data_to_send, 7);
}

/////////////////////////////////////////////////////////////////////////////////////
//Data_Receive_Prepare������Э��Ԥ����������Э��ĸ�ʽ�����յ������ݽ���һ�θ�ʽ�Խ�������ʽ��ȷ�Ļ��ٽ������ݽ���
//��ֲʱ���˺���Ӧ���û���������ʹ�õ�ͨ�ŷ�ʽ���е��ã����紮��ÿ�յ�һ�ֽ����ݣ�����ô˺���һ��
//�˺������������ϸ�ʽ������֡�󣬻����е������ݽ�������
void ANO_DT_Data_Receive_Prepare(u8 data)
{
	static u8 RxBuffer[50];
	static u8 _data_len = 0,_data_cnt = 0;
	static u8 state = 0;
	
	if(state==0&&data==0xAA)
	{
		state=1;
		RxBuffer[0]=data;
	}
	else if(state==1&&data==0xAF)
	{
		state=2;
		RxBuffer[1]=data;
	}
	else if(state==2&&data<0XF1)
	{
		state=3;
		RxBuffer[2]=data;
	}
	else if(state==3&&data<50)
	{
		state = 4;
		RxBuffer[3]=data;
		_data_len = data;
		_data_cnt = 0;
	}
	else if(state==4&&_data_len>0)
	{
		_data_len--;
		RxBuffer[4+_data_cnt++]=data;
		if(_data_len==0)
			state = 5;
	}
	else if(state==5)
	{
		state = 0;
		RxBuffer[4+_data_cnt]=data;
		ANO_DT_Data_Receive_Anl(RxBuffer,_data_cnt+5);
	}
	else
		state = 0;
}
/////////////////////////////////////////////////////////////////////////////////////
//Data_Receive_Anl������Э�����ݽ������������������Ƿ���Э���ʽ��һ������֡���ú��������ȶ�Э�����ݽ���У��
//У��ͨ��������ݽ��н�����ʵ����Ӧ����
//�˺������Բ����û����е��ã��ɺ���Data_Receive_Prepare�Զ�����
void ANO_DT_Data_Receive_Anl(u8 *data_buf,u8 num)
{
	u8 ADD = 0;
	u8 i=0;
	for( i=0;i<(num-1);i++)
		ADD += *(data_buf+i);
	if(!(ADD==*(data_buf+num-1)))		return;		//�ж�sum
	if(!(*(data_buf)==0xAA && *(data_buf+1)==0xAF))		return;		//�ж�֡ͷ
	
	if(*(data_buf+2)==0X01)
	{
//		if(*(data_buf+4)==0X01)
//			mpu6050.Acc_CALIBRATE = 1;
//		if(*(data_buf+4)==0X02)
//			mpu6050.Gyro_CALIBRATE = 1;
//		if(*(data_buf+4)==0X03)
//		{
//			mpu6050.Acc_CALIBRATE = 1;		
//			mpu6050.Gyro_CALIBRATE = 1;			
//		}
	}
	
	if(*(data_buf+2)==0X02)
	{
		if(*(data_buf+4)==0X01)
		{
			f.send_pid1 = 1;
			f.send_pid2 = 1;
			f.send_pid3 = 1;
			f.send_pid4 = 1;
			f.send_pid5 = 1;
			f.send_pid6 = 1;
		}
		if(*(data_buf+4)==0X02)
		{
			
		}
		if(*(data_buf+4)==0XA0)		//��ȡ�汾��Ϣ
		{
//			f.send_version = 1;
		}
		if(*(data_buf+4)==0XA1)		//�ָ�Ĭ�ϲ���
		{
		//	Para_ResetToFactorySetup();
		}
	}

	if(*(data_buf+2)==0X10)								//PID1//�ڻ�����
    {
//			Test[0]   = ( (vs16)(*(data_buf+4)<<8)|*(data_buf+5) );
//		  Test[1]   =( (vs16)(*(data_buf+6)<<8)|*(data_buf+7) );
//			Test[2]   = ( (vs16)(*(data_buf+8)<<8)|*(data_buf+9) );
//			Test[3]   =   ((vs16)(*(data_buf+10)<<8)|*(data_buf+11) );
			
//				Wind_W_Pid.X_Kp = ((vs16)(*(data_buf+4)<<8)|*(data_buf+5) );
//				Wind_W_Pid.X_Ki = ( (vs16)(*(data_buf+6)<<8)|*(data_buf+7) );
//        Wind_W_Pid.X_Kd = ( (vs16)(*(data_buf+8)<<8)|*(data_buf+9) );
//        Wind_W_Pid.Y_Kp = ( (vs16)(*(data_buf+10)<<8)|*(data_buf+11) );
//        Wind_W_Pid.Y_Ki = ( (vs16)(*(data_buf+12)<<8)|*(data_buf+13) );
//        Wind_W_Pid.Y_Kd = ( (vs16)(*(data_buf+14)<<8)|*(data_buf+15) );
//			
//        Set_Range 	    = ( (vs16)(*(data_buf+16)<<8)|*(data_buf+17) )/1000;
//        Set_ErrorAngle	= ((vs16)(*(data_buf+18)<<8)|*(data_buf+19) );
//				_Inner_Control.Yaw.Kd 	= ( (vs16)(*(data_buf+20)<<8)|*(data_buf+21) );
				ANO_DT_Send_Check(*(data_buf+2),ADD);
//				Param_SavePID();
    }
    if(*(data_buf+2)==0X11)								//PID2
    {
//				_Out_Control.Roll.Kp  = ( (vs16)(*(data_buf+4)<<8)|*(data_buf+5) );
//				_Out_Control.Roll.Ki  = ( (vs16)(*(data_buf+6)<<8)|*(data_buf+7) );
//        _Out_Control.Roll.Kd  = ( (vs16)(*(data_buf+8)<<8)|*(data_buf+9) );
//       	_Out_Control.Pitch.Kp = ( (vs16)(*(data_buf+10)<<8)|*(data_buf+11) );
//        _Out_Control.Pitch.Ki = ( (vs16)(*(data_buf+12)<<8)|*(data_buf+13) );
//        _Out_Control.Pitch.Kd = ( (vs16)(*(data_buf+14)<<8)|*(data_buf+15) );
//        _Out_Control.Yaw.Kp 	= ( (vs16)(*(data_buf+16)<<8)|*(data_buf+17) );
//        _Out_Control.Yaw.Ki		= ( (vs16)(*(data_buf+18)<<8)|*(data_buf+19) );
//				_Out_Control.Yaw.Kd 	= ( (vs16)(*(data_buf+20)<<8)|*(data_buf+21) );
					
        ANO_DT_Send_Check(*(data_buf+2),ADD);
			//	Param_SavePID();
    }
    if(*(data_buf+2)==0X12)								//PID3
    {	
//				thr = ( (vs16)(*(data_buf+4)<<8)|*(data_buf+5) );
//				Flag_ThrControl.Fly_ready	= ( (vs16)(*(data_buf+6)<<8)|*(data_buf+7) );
      //  _Out_Control.Roll.Expect = ( (vs16)(*(data_buf+8)<<8)|*(data_buf+9) );
			//_Out_Control.Pitch.Expect = ( (vs16)(*(data_buf+10)<<8)|*(data_buf+11) );
//        	Thr_Low = ( (vs16)(*(data_buf+10)<<8)|*(data_buf+11) );
//					Thr_High= ( (vs16)(*(data_buf+12)<<8)|*(data_buf+13) );
//					_Out_Control.Roll.Expect  = (vs16)(*(data_buf+14)<<8)|*(data_buf+15);
//          _Out_Control.Pitch.Expect = ( (vs16)(*(data_buf+16)<<8)|*(data_buf+17) );
//			    _Out_Control.Yaw.Expect	  = ( (vs16)(*(data_buf+18)<<8)|*(data_buf+19) );
//        trl_2.PID[PIDYAW].kd 	= 0 .001*( (vs16)(*(data_buf+20)<<8)|*(data_buf+21) );
					ANO_DT_Send_Check(*(data_buf+2),ADD);
				//Param_SavePID();
    }
	if(*(data_buf+2)==0X13)								//PID4
	{
		ANO_DT_Send_Check(*(data_buf+2),ADD);
	}
	if(*(data_buf+2)==0X14)								//PID5
	{
		ANO_DT_Send_Check(*(data_buf+2),ADD);
	}
	if(*(data_buf+2)==0X15)								//PID6
	{
		ANO_DT_Send_Check(*(data_buf+2),ADD);
	}
}

void ANO_DT_Send_Version(u8 hardware_type, u16 hardware_ver,u16 software_ver,u16 protocol_ver,u16 bootloader_ver)
{
	u8 _cnt=0;
	u8 ADD = 0;
	u8 i=0;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0x00;
	data_to_send[_cnt++]=0;
	
	data_to_send[_cnt++]=hardware_type;
	data_to_send[_cnt++]=BYTE1(hardware_ver);
	data_to_send[_cnt++]=BYTE0(hardware_ver);
	data_to_send[_cnt++]=BYTE1(software_ver);
	data_to_send[_cnt++]=BYTE0(software_ver);
	data_to_send[_cnt++]=BYTE1(protocol_ver);
	data_to_send[_cnt++]=BYTE0(protocol_ver);
	data_to_send[_cnt++]=BYTE1(bootloader_ver);
	data_to_send[_cnt++]=BYTE0(bootloader_ver);
	
	data_to_send[3] = _cnt-4;
	

	for(i=0;i<_cnt;i++)
		ADD += data_to_send[i];
	data_to_send[_cnt++]=ADD;
	
	ANO_DT_Send_Data(data_to_send, _cnt);
}
void ANO_DT_Send_Status(float angle_rol, float angle_pit, float angle_yaw, s32 alt, u8 fly_model, u8 armed)
{
	u8 _cnt=0;
	vs16 _temp;
	vs32 _temp2 = alt;
	u8 ADD = 0;
	u8 i=0;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0x01;
	data_to_send[_cnt++]=0;
	
	_temp = (int)(angle_rol*100);
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = (int)(angle_pit*100);
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = (int)(angle_yaw*100);
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	
	data_to_send[_cnt++]=BYTE3(_temp2);
	data_to_send[_cnt++]=BYTE2(_temp2);
	data_to_send[_cnt++]=BYTE1(_temp2);
	data_to_send[_cnt++]=BYTE0(_temp2);
	
	data_to_send[_cnt++] = fly_model;
	
	data_to_send[_cnt++] = armed;
	
	data_to_send[3] = _cnt-4;
	

	for(i=0;i<_cnt;i++)
		ADD += data_to_send[i];
	data_to_send[_cnt++]=ADD;
	
	ANO_DT_Send_Data(data_to_send, _cnt);
}
void ANO_DT_Send_Senser(s16 a_x,s16 a_y,s16 a_z,s16 g_x,s16 g_y,s16 g_z,s16 m_x,s16 m_y,s16 m_z,s32 bar)
{
	u8 _cnt=0;
	vs16 _temp;
	u8 ADD = 0;
	u8 i=0;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0x02;
	data_to_send[_cnt++]=0;
	
	_temp = a_x;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = a_y;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = a_z;	
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	
	_temp = g_x;	
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = g_y;	
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = g_z;	
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	
	_temp = m_x;	
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = m_y;	
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = m_z;	
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	
	data_to_send[3] = _cnt-4;
	
	
	
	for(i=0;i<_cnt;i++)
		ADD += data_to_send[i];
	data_to_send[_cnt++] = ADD;
	
	ANO_DT_Send_Data(data_to_send, _cnt);
}
void ANO_DT_Send_RCData(u16 thr,u16 yaw,u16 rol,u16 pit,u16 aux1,u16 aux2,u16 aux3,u16 aux4,u16 aux5,u16 aux6)
{
	u8 _cnt=0;
	u8 ADD = 0;
	u8 i=0;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0x03;
	data_to_send[_cnt++]=0;
	data_to_send[_cnt++]=BYTE1(thr);
	data_to_send[_cnt++]=BYTE0(thr);
	data_to_send[_cnt++]=BYTE1(yaw);
	data_to_send[_cnt++]=BYTE0(yaw);
	data_to_send[_cnt++]=BYTE1(rol);
	data_to_send[_cnt++]=BYTE0(rol);
	data_to_send[_cnt++]=BYTE1(pit);
	data_to_send[_cnt++]=BYTE0(pit);
	data_to_send[_cnt++]=BYTE1(aux1);
	data_to_send[_cnt++]=BYTE0(aux1);
	data_to_send[_cnt++]=BYTE1(aux2);
	data_to_send[_cnt++]=BYTE0(aux2);
	data_to_send[_cnt++]=BYTE1(aux3);
	data_to_send[_cnt++]=BYTE0(aux3);
	data_to_send[_cnt++]=BYTE1(aux4);
	data_to_send[_cnt++]=BYTE0(aux4);
	data_to_send[_cnt++]=BYTE1(aux5);
	data_to_send[_cnt++]=BYTE0(aux5);
	data_to_send[_cnt++]=BYTE1(aux6);
	data_to_send[_cnt++]=BYTE0(aux6);

	data_to_send[3] = _cnt-4;
	

	for(i=0;i<_cnt;i++)
		ADD += data_to_send[i];
	
	data_to_send[_cnt++]=ADD;
	
	ANO_DT_Send_Data(data_to_send, _cnt);
}
void ANO_DT_Send_Power(u16 votage, u16 current)
{
	u8 _cnt=0;
	u16 temp;
	u8 ADD = 0;
	u8 i=0;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0x05;
	data_to_send[_cnt++]=0;
	
	temp = votage;
	data_to_send[_cnt++]=BYTE1(temp);
	data_to_send[_cnt++]=BYTE0(temp);
	temp = current;
	data_to_send[_cnt++]=BYTE1(temp);
	data_to_send[_cnt++]=BYTE0(temp);
	
	data_to_send[3] = _cnt-4;
	

	for(i=0;i<_cnt;i++)
		ADD += data_to_send[i];
	
	data_to_send[_cnt++]=ADD;
	
	ANO_DT_Send_Data(data_to_send, _cnt);
}
void ANO_DT_Send_MotoPWM(u16 m_1,u16 m_2,u16 m_3,u16 m_4,u16 m_5,u16 m_6,u16 m_7,u16 m_8)
{
	u8 _cnt=0;
	u8 ADD = 0;
	u8 i=0;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0x06;
	data_to_send[_cnt++]=0;
	
	data_to_send[_cnt++]=BYTE1(m_1);
	data_to_send[_cnt++]=BYTE0(m_1);
	data_to_send[_cnt++]=BYTE1(m_2);
	data_to_send[_cnt++]=BYTE0(m_2);
	data_to_send[_cnt++]=BYTE1(m_3);
	data_to_send[_cnt++]=BYTE0(m_3);
	data_to_send[_cnt++]=BYTE1(m_4);
	data_to_send[_cnt++]=BYTE0(m_4);
	data_to_send[_cnt++]=BYTE1(m_5);
	data_to_send[_cnt++]=BYTE0(m_5);
	data_to_send[_cnt++]=BYTE1(m_6);
	data_to_send[_cnt++]=BYTE0(m_6);
	data_to_send[_cnt++]=BYTE1(m_7);
	data_to_send[_cnt++]=BYTE0(m_7);
	data_to_send[_cnt++]=BYTE1(m_8);
	data_to_send[_cnt++]=BYTE0(m_8);
	
	data_to_send[3] = _cnt-4;

	for(i=0;i<_cnt;i++)
		ADD += data_to_send[i];
	
	data_to_send[_cnt++]=ADD;
	
	ANO_DT_Send_Data(data_to_send, _cnt);
}
void ANO_DT_Send_PID(u8 group,float p1_p,float p1_i,float p1_d,float p2_p,float p2_i,float p2_d,float p3_p,float p3_i,float p3_d)
{
	u8 _cnt=0;
	vs16 _temp;
	u8 ADD = 0;
	u8 i=0;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0x10+group-1;
	data_to_send[_cnt++]=0;
	
	
	_temp = p1_p * 1000;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = p1_i  * 1000;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = p1_d  * 1000;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = p2_p  * 1000;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = p2_i  * 1000;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = p2_d * 1000;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = p3_p  * 1000;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = p3_i  * 1000;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = p3_d * 1000;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	
	data_to_send[3] = _cnt-4;
	
	
	for(i=0;i<_cnt;i++)
		ADD += data_to_send[i];
	
	data_to_send[_cnt++]=ADD;

	ANO_DT_Send_Data(data_to_send, _cnt);
}



void ANO_DT_Send_Mypara(float para1,float para2,float para3,float para4,float para5,float para6,float para7,float para8,float para9)
{
	u8 _cnt=0;
	u8 ADD = 0;
	u8 i=0;
	vs16 _temp = para1*100;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xF1;
	data_to_send[_cnt++]=0;
	
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = para2*100;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = para3*100;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = para4*100;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = para5*100;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = para6*100;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = para7*100;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = para8*100;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = para9*100;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	
	data_to_send[3] = _cnt-4;
	

	for(i=0;i<_cnt;i++)
		ADD += data_to_send[i];
	
	data_to_send[_cnt++]=ADD;
	
	ANO_DT_Send_Data(data_to_send, _cnt);
}
