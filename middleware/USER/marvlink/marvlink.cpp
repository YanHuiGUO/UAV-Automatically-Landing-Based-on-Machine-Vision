#include "include.h"					
const int MAV_GCS_sysid  = 255;
const	int MAV_GCS_WP_comid =  MAV_COMP_ID_MISSIONPLANNER ;
const	int MAV_APM_sysid = 1;
const int MAV_APM_comid = 1;
msgGroup mavMsg;
marvlink_func mavMsgFunc;
marvlink_flag mavMsgFlag;

//MESSGE CRC数组，和MAVLINK_MSG_ID适配
const UINT_8 message_info[MAV_MESSAGE_LEN]={
			50   , //HEARTBEAT  0
	    124  , //SYS_STATUS  1
			137  , //SYSTEM_TIME  2
	    0    , //保留位  3
			237  , //PING  4
			217  , //CHANGE_OPERATOR_CONTROL 5
			104  , //CHANGE_OPERATOR_CONTROL_ACK 6
			119  , //AUTH_KEY 7
			0,0,0, //保留 8,9,10
			89   , //SET_MODE 11
			0,0,0,0,0,0,0,0, //保留 12,13,14,15,16,17,18,19
			214  , //PARAM_REQUEST_READ 20
		  159  , //PARAM_REQUEST_LIST 21
			220  , //PARAM_VALUE 22
    	168  , //PARAM_SET 23
	    24   , //GPS_RAW_INT 24
	    23   , //GPS_STATUS 25
			170  , //SCALED_IMU 26
		  144  , //RAW_IMU 27
    	67   , //RAW_PRESSURE 28
	    115  , //SCALED_PRESSURE 29
	    39   , //ATTITUDE 30
			246  , //ATTITUDE_QUATERNION 31
			185  , //LOCAL_POSITION_NED 32
			104  , //GLOBAL_POSITION_INT 33
			237  , //RC_CHANNELS_SCALED 34
			244  , //RC_CHANNELS_RAW 35
			222  , //SERVO_OUTPUT_RAW 36
			212  , //MISSION_REQUEST_PARTIAL_LIST 37
			9    , //MISSION_WRITE_PARTIAL_LIST 38
			254  , //MISSION_ITEM 39
			230  , //MISSION_REQUEST 40 
			28   , //MISSION_SET_CURRENT 41
			28   , //MISSION_CURRENT 42
			132  , //MISSION_REQUEST_LIST 43
			221  , //MISSION_COUNT 44
			232  , //MISSION_CLEAR_ALL 45
			11   , //MISSION_ITEM_REACHED 46
			153  , //MISSION_ACK 47
			41   , //SET_GPS_GLOBAL_ORIGIN 48
			39   , //GPS_GLOBAL_ORIGIN 49
			78   , //PARAM_MAP_RC 50
			196  , //MISSION_REQUEST_INT 51
			0,0  , //保留 52,53
			15   , //SAFETY_SET_ALLOWED_AREA 54
			3    , //SAFETY_ALLOWED_AREA 55
			0,0,0,0,0, //保留 56,57,58,59,60
			153  , //ATTITUDE_QUATERNION_COV 61
			183  , //NAV_CONTROLLER_OUTPUT 62
			51   , //GLOBAL_POSITION_INT_COV 63
			59   , //LOCAL_POSITION_NED_COV 64
			118  , //RC_CHANNELS 65
			148  , //REQUEST_DATA_STREAM 66
			21   , //DATA_STREAM 67
			0    , //68
			243  , //MANUAL_CONTROL 69
			124  , //RC_CHANNELS_OVERRIDE 70
			0,0  , //71,72
			38   , //MISSION_ITEM_INT 73
			20   , //VFR_HUD 74
			158  , //COMMAND_INT 75
			152  , //COMMAND_LONG 76
			143  , //COMMAND_ACK 77
			0,0,0, //78,79,80
			106  , //MANUAL_SETPOINT 81
			49   , //SET_ATTITUDE_TARGET 82
			22   , //ATTITUDE_TARGET 83
			143  , //SET_POSITION_TARGET_LOCAL_NED 84
			140  , //POSITION_TARGET_LOCAL_NED 85
			5    , //SET_POSITION_TARGET_GLOBAL_INT 86
			150  , //POSITION_TARGET_GLOBAL_INT 87
			0    , //88
			231  , //LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET 89
			183  , //HIL_STATE 90
			63   , //HIL_CONTROLS 91
			54   , //HIL_RC_INPUTS_RAW 92 
			0,0,0,0,0,0,0, //93,94,95,96,97,98,99
			175  , //OPTICAL_FLOW 100
			102  , //GLOBAL_VISION_POSITION_ESTIMATE 101
			158  , //VISION_POSITION_ESTIMATE 102
			208  , //VISION_SPEED_ESTIMATE 103
			56   , //VICON_POSITION_ESTIMATE 104
			93   , //HIGHRES_IMU 105
			138  , //OPTICAL_FLOW_RAD 106
			108  , //HIL_SENSOR 107
			32   , //SIM_STATE 108
			185  , //RADIO_STATUS 109
			84   , //FILE_TRANSFER_PROTOCOL 110
			34   , //TIMESYNC 111
			174  , //CAMERA_TRIGGER 112
			124  , //HIL_GPS 113
			237  , //HIL_OPTICAL_FLOW 114
			4    , //HIL_STATE_QUATERNION 115
			76   , //SCALED_IMU2 116
			128  , //LOG_REQUEST_LIST 117 
			56   , //LOG_ENTRY 118
			116  , //LOG_REQUEST_DATA 119
			134  , //LOG_DATA 120
			237  , //LOG_ERASE 121
			203  , //LOG_REQUEST_END 122
			250  , //GPS_INJECT_DATA 123
			87   , //GPS2_RAW 124
			203  , //POWER_STATUS 125
			220  , //SERIAL_CONTROL 126
			25   , //GPS_RTK 127
			226  , //GPS2_RTK 128
			46   , //SCALED_IMU3 129
			29   , //DATA_TRANSMISSION_HANDSHAKE 130
			223  , //ENCAPSULATED_DATA 131
			85   , //DISTANCE_SENSOR 132
			6    , //TERRAIN_REQUEST 133
			229  , //TERRAIN_DATA 134
			203  , //TERRAIN_CHECK 135
			1    , //TERRAIN_REPORT 136
			195  , //SCALED_PRESSURE2 137
			109  , //ATT_POS_MOCAP 138
			168  , //SET_ACTUATOR_CONTROL_TARGET 139
			181  , //ACTUATOR_CONTROL_TARGET 140
			47   , //ALTITUDE 141
			72   , //RESOURCE_REQUEST 142
			131  , //SCALED_PRESSURE3 143
			127  , //FOLLOW_TARGET 144
			0    , //145
			103  , //CONTROL_SYSTEM_STATE 146
			154  , //BATTERY_STATUS 147
			178  , //AUTOPILOT_VERSION 148
			200  , //LANDING_TARGET 149
			134  , //SENSOR_OFFSETS 150
			219  , //SET_MAG_OFFSETS 151
			208  , //MEMINFO 152
			188  , //AP_ADC 153
			84   , //DIGICAM_CONFIGURE 154
			22   , //DIGICAM_CONFIGURE 155
			19   , //MOUNT_CONFIGURE 156
			21   , //MOUNT_CONTROL 157
			134  , //MOUNT_STATUS 158
			0    , //159
			78   , //FENCE_POINT 160
			68   , //FENCE_FETCH_POINT 161
			189  , //FENCE_STATUS 162
			127  , //AHRS 163
			154  , //SIMSTATE 164
			21   , //HWSTATUS 165
			21   , //RADIO 166
			144  , //LIMITS_STATUS 167
			1    , //WIND 168
			234  , //DATA16 169
			73   , //DATA32 170
			181  , //DATA64 171
			22   , //DATA96 172
			83   , //RANGEFINDER 173
			167  , //AIRSPEED_AUTOCAL 174
			138  , //RALLY_POINT 175
			234  , //RALLY_FETCH_POINT 176
			240  , //COMPASSMOT_STATUS 177
			47   , //AHRS2 178
			189  , //CAMERA_STATUS 179
			52   , //CAMERA_FEEDBACK 180
			174  , //BATTERY2 181
			229  , //AHRS3 182
			85   , //AUTOPILOT_VERSION_REQUEST 183
			159  , //REMOTE_LOG_DATA_BLOCK 184
			186  , //REMOTE_LOG_BLOCK_STATUS 185
			72   , //LED_CONTROL 186
			0,0,0,0, // 187,188,189,190
			92   , //MAG_CAL_PROGRESS 191
			36   , //MAG_CAL_REPORT 192
			71   , //EKF_STATUS_REPORT 193
			98   , //PID_TUNING 194
			0,0,0,0,0, //195,196,197,198,199
			134  , //GIMBAL_REPORT 200
			205  , //GIMBAL_CONTROL 201
			0,0,0,0,0,0,0,0,0,0,0,0,//202,203,204,205,206,207,208,209,210,211,212,213
			69   , //GIMBAL_TORQUE_CMD_REPORT 214
			101  , //GOPRO_HEARTBEAT 215
			50   , //GOPRO_GET_REQUEST 216
			202  , //GOPRO_GET_RESPONSE 217
			17   , //GOPRO_SET_REQUEST 218
			162  , //GOPRO_SET_RESPONSE 219
			0,0,0,0,0,0,//220,221,222,223,224,225
			207  , //RPM 226
			0,0,0,//227,228,229
			163  , //ESTIMATOR_STATUS 230
			105  , //WIND_COV 231
			0    , //232
			35   , //GPS_RTCM_DATA 233
			0,0,0,0,0,0,0, // 234,235,236,237,238,239,240
			90   , //VIBRATION 241
			104  , //HOME_POSITION 242
			85   , //SET_HOME_POSITION 243
			95   , //MESSAGE_INTERVAL 244
			130  , //EXTENDED_SYS_STATE 245
			184  , //ADSB_VEHICLE 246
			0    , //247
			8    , //V2_EXTENSION 248
			204  , //MEMORY_VECT 249
			49   , //DEBUG_VECT 250
			170  , //NAMED_VALUE_FLOAT 251
			44   , //NAMED_VALUE_INT 252
			83   , //STATUSTEXT 253
			46   , //DEBUG 254
			0    , // 255
			71   , //SETUP_SIGNING 256
};


/******************************************/
/*解析部分
/******************************************/

//解析ATTITUDE /mavlink_attitude_t
mavlink_attitude_t marvlink_func::Analyse_ATTITUDE(uint8_t *buffer)
{
	//Size=28
		UINT_8 msgbuffer[28];
		mavlink_attitude_t mymsg;
//	union
//	{
//		mavlink_attitude_t mymsg;
//		UINT_8 msgbuffer[28];
//	}a;//32是小端，可以直接union,但是最好不要
	
	//验证一下buffer[1]的大小和结构体的字节数量是否一致
	INT_32 bufferStructSize =sizeof(mavlink_attitude_t);
	if(bufferStructSize == buffer[1])
	{
		for (int i = 0; i < bufferStructSize; i++)
			*(msgbuffer + i) = buffer[6 + i];
		 mymsg = *(mavlink_attitude_t *)msgbuffer;
		 //return 1;
	}
	 return mymsg;
}



//解析POSITION_INT /mavlink_global_position_int_t

float lat,lng=0;

//mavlink_global_position_int_t marvlink_func::Analyse_GLOBAL_POSITION_INT(uint8_t *buffer)
//{
//	//Size=28
//	UINT_8 msgbuffer[28];
//	INT_32 bufferStructSize =sizeof(mavlink_global_position_int_t);
//	mavlink_global_position_int_t mymsg;
//	if(bufferStructSize == buffer[1])
//	{
//		for (int i = 0; i < bufferStructSize; i++)
//			*(msgbuffer + i) = buffer[6 + i];
//		 mymsg = *(mavlink_global_position_int_t *)msgbuffer;
//	}
//	
//	WP_TASK.lat = mymsg.lat/1.0e7;
//	WP_TASK.lng = mymsg.lon/1.0e7;
//	WP_TASK.alt = mymsg.alt/1000;
//	return mymsg;
//}
//解析出地面站第一页显示的信息 /mavlink_vfr_hud_t
mavlink_vfr_hud_t marvlink_func::Analyse_VFR_HUD(uint8_t *buffer)
{

	//Size=20
	UINT_8 msgbuffer[20];
	INT_32 bufferStructSize =sizeof(mavlink_vfr_hud_t);
  mavlink_vfr_hud_t mymsg;
	if(bufferStructSize == buffer[1])
	{
		for (int i = 0; i < bufferStructSize; i++)
			*(msgbuffer + i) = buffer[6 + i];
	  mymsg = *(mavlink_vfr_hud_t *)msgbuffer;

	}
	 return mymsg;
}
mavlink_mission_ack_t marvlink_func::Analyse_mission_ack_t(uint8_t *buffer)
{
	//Size=3
	UINT_8 msgbuffer[sizeof(mavlink_mission_ack_t)];
	INT_32 bufferStructSize =sizeof(mavlink_mission_ack_t);
  mavlink_mission_ack_t mymsg;
	if(bufferStructSize == buffer[1])
	{
		for (int i = 0; i < bufferStructSize; i++)
			*(msgbuffer + i) = buffer[6 + i];
	  mymsg = *(mavlink_mission_ack_t *)msgbuffer;

	}
	 return mymsg;
}
mavlink_mission_request_t marvlink_func::Analyse_mission_request_t(uint8_t *buffer)
{
	//Size=3
	UINT_8 msgbuffer[sizeof(mavlink_mission_request_t)];
	INT_32 bufferStructSize =sizeof(mavlink_mission_request_t);
  mavlink_mission_request_t mymsg;
	if(bufferStructSize == buffer[1])
	{
		for (int i = 0; i < bufferStructSize; i++)
			*(msgbuffer + i) = buffer[6 + i];
	  mymsg = *(mavlink_mission_request_t *)msgbuffer;
	}
	 return mymsg;
}

/******************************************/
/*发送组帧部分
/******************************************/
int packetcount = 0; 
template <typename T>
void marvlink_func::generatePacket(int messageType, T indata, int sysid, int compid, bool forcemavlink2 , bool forcesigning)
{

		 UINT_8 packet[sizeof(indata)+ 6 + 2];
		 UINT_8 data_len = sizeof(indata);
		 UINT_8 *data;
	 
		 int i;

		 T msgbuffer = indata;
		 data = (UINT_8 *)&msgbuffer;	

		 packet[0] = MAVLINK_STX_MAVLINK1;
     packet[1] = (UINT_8) data_len;
     packet[2] = (UINT_8) packetcount;

     packetcount++;

     packet[3] = (UINT_8)MAV_GCS_sysid;
     packet[4] = (UINT_8)MAV_COMP_ID_MISSIONPLANNER;
     packet[5] = (UINT_8)messageType;

		
	  for (i= 6;i<data_len+6;i++)
				packet[i] = data[i-6];

		UINT_16 checksum = crc_calculate(packet, packet[1] + 6);

		//
		checksum = crc_accumulate(message_info[messageType], checksum);

    UINT_8 ck_a = (UINT_8) (checksum & 0xFF); ///< High byte
    UINT_8 ck_b = (UINT_8) (checksum >> 8); ///< Low byte

    packet[i] = ck_a;
    i += 1;
    packet[i] = ck_b;
    i += 1;
		
	  INSERT_SETWP(packet,data_len+ 8);
}
//设置航点数目
void marvlink_func::setWPTotal(UINT_16 wp_total)
{
		mavlink_mission_count_t req ;

		req.target_system = MAV_APM_sysid;
		req.target_component = MAV_APM_comid; 

		req.count = wp_total;
	
		//下面是发送指令的固定格式
		if(0 == mavMsgFlag.WpCommond.S_2 )
			generatePacket((UINT_8)MISSION_COUNT, req);
		else if(1 == mavMsgFlag.WpCommond.S_2 )
		{			
			mavMsgFlag.WpCommond.S_2 =0;	
		}
}



MAV_MISSION_RESULT marvlink_func::setWP(mavlink_mission_item_t req)
 {
		UINT_16  index;
		index = req.seq;
		// request					
	 
		//下面是发送指令的固定格式
		if(0 == mavMsgFlag.WpCommond.S_2 )
			generatePacket((byte)MISSION_ITEM, req);
		else if(1 == mavMsgFlag.WpCommond.S_2 )
		{			
			mavMsgFlag.WpCommond.S_2 =0;		
			return 	(MAV_MISSION_RESULT) mavMsgFlag.AnalyResult;
		}
		
		return MAV_MISSION_DENIED;//未接收到响应
 }
 
 
//设置航点 使用整型来设置航点
MAV_MISSION_RESULT marvlink_func::setWP(Locationwp loc, UINT_16 index, MAV_FRAME frame, UINT_8 current,
            UINT_8 autocontinue , bool use_int )
{
			mavlink_mission_item_t req ;

			req.target_system = MAV_APM_sysid;
			req.target_component = MAV_APM_comid;

			req.command = loc.id;

			req.current = current;
			req.autocontinue = autocontinue;

			req.frame = (byte) frame;
			req.y = loc.lng;
			req.x = loc.lat;
			req.z = (float) (loc.alt);

			req.param1 = loc.p1;
			req.param2 = loc.p2;
			req.param3 = loc.p3;
			req.param4 = loc.p4;

			req.seq = index;

			return setWP(req);
}

void marvlink_func::setWPACK()
{
	mavlink_mission_ack_t req;
	req.target_system = MAV_APM_sysid;
	req.target_component = MAV_APM_comid;
	req.type = 0;
	if(0 == mavMsgFlag.WpCommond.S_2 )
			generatePacket((byte)MISSION_ACK, req);
	if(1 == mavMsgFlag.WpCommond.S_2 ) 
			mavMsgFlag.WpCommond.S_2 = 0;

}
bool marvlink_func::doCommand(MAV_CMD actionid, float p1, float p2, float p3, float p4, float p5, float p6, float p7)
{
//     MAVLinkMessage buffer;
			mavlink_command_long_t req;

			req.target_system = MAV_APM_sysid;
			req.target_component = MAV_APM_comid;

			req.command = (ushort) actionid;

			req.param1 = p1;
			req.param2 = p2;
			req.param3 = p3;
			req.param4 = p4;
			req.param5 = p5;
			req.param6 = p6;
			req.param7 = p7;

			if(0 == mavMsgFlag.WpCommond.S_2 )
					generatePacket((byte)COMMAND_LONG, req);
			else if(1 == mavMsgFlag.WpCommond.S_2 )
					return true;
}



void marvlink_func::setMode(mavlink_set_mode_t mode)
{
	if(0 == mavMsgFlag.WpCommond.S_2 )
			generatePacket((byte)SET_MODE, mode);
	else if(1 == mavMsgFlag.WpCommond.S_2 )
		{			
			mavMsgFlag.WpCommond.S_2 =0;		
		}
}

bool marvlink_func::setGuidedModeWP(Locationwp gotohere, bool setguidedmode)
 {

		if (gotohere.alt == 0 || gotohere.lat == 0 || gotohere.lng == 0)
				return false;
	
		gotohere.id = (ushort)WAYPOINT;

//		if (setguidedmode)
//				{
//						// fix for followme change                
//							
//				}

		MAV_MISSION_RESULT ans = setWP(gotohere, 0,GLOBAL_RELATIVE_ALT, (byte) 2,1,0);

				if (ans != MAV_MISSION_ACCEPTED)
					return false;
				
		return true;

}



//控制遥控器通道值,注意通过此函数写的值不能由遥控器更改了，除非复位
bool marvlink_func::Write_RCoverride(void)
{
	
		mavlink_rc_channels_override_t rc;
		rc.target_component = MAV_APM_sysid;
		rc.target_system = MAV_APM_comid;

//		rc.chan1_raw = 1600;
//		rc.chan2_raw = 0;
//		rc.chan3_raw = 1540;
//		rc.chan4_raw = 0;
//		rc.chan5_raw = 0;
//		rc.chan6_raw = 0;
//		rc.chan7_raw = 0;
//		rc.chan8_raw = 0;

		generatePacket(RC_CHANNELS_OVERRIDE , rc);
		return true;
}


//插入PC->APM发送链中，发送解析板指令
void marvlink_func::INSERT_SETWP(UINT_8 *buffer,UINT_32 bufferlen)
{
//		uint32_t temp = 0;	
//		temp = bufferlen;
//  
//		for (int i = 0;i < temp;i++)   
//				PC_SEND2_APM[i] = buffer[i]; 
//	
//		//先切断PC->APM通信链
//  	mavMsgFlag.WpCommond.S_1 = 1;	
//	
//		if(mavMsgFlag.Flag_sendToAPM == 0)
//		{			
//				DMA_SetCurrDataCounter(DMA1_Channel2,temp);	
//				DMA_Cmd(DMA1_Channel2, ENABLE);
//				mavMsgFlag.Flag_sendToAPM = 1;			
//		}
//		DMA_SetCurrDataCounter(DMA2_Channel5,temp);
//		DMA_Cmd(DMA2_Channel5, ENABLE); 
		
}


	
	
	
	