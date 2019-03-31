#include <include.h>
#define Flow_Len 25
para_flow Para_Flow;
void Get_Flow_Data(UINT_8* arr,int valid_data_len){
		Flow_Step step=GROY_x;
		for (int i = 0; i < valid_data_len-1; i = i + 4)
				{
					unsigned char tempdata[4] = { 0 };
					for (int j = 0; j < 4; j++)
					{
						tempdata[j] = arr[i+j];
					}
					switch (step)
					{
					case GROY_x : Para_Flow.groy_x= apmMAV.Byte2Float(tempdata);
														step = GROY_y ;
														break;
					case GROY_y :	Para_Flow.groy_y = apmMAV.Byte2Float(tempdata);
														step = Sonar_Flow ;
														break;
					case Sonar_Flow ://Para_APM.ACC_z =apmMAV.Byte2Float(tempdata);
							
														step = Flow_comp_x ; 
														break;
					case Flow_comp_x :Para_Flow.flow_comp_x = apmMAV.Byte2Float(tempdata);
								
														step = Flow_comp_y ;
														break;
					case Flow_comp_y : Para_Flow.flow_comp_y = apmMAV.Byte2Float(tempdata);
														step = GROY_x ;
														break;
					}
				}
				
		//注意光流算出的方向和定义的APM的y，x方向是相同
		//对于光流的y方向,左旋会使光流增，右旋会使光流值减 	//x方向，前旋转减，后旋转增
		//				|x			
		//				|
					//-----y
//	Para_pos.v_x  = Para_Flow.flow_comp_x - Para_Flow.groy_y*Para_Flow.k_comp_x;
//	Para_pos.v_y = Para_Flow.flow_comp_y + Para_Flow.groy_x*Para_Flow.k_comp_y;	
//				
//	Para_pos.v_x  *=Sonar.UltraHeight*cos(Para_APM.Angle_pitch)*cos(Para_APM.Angle_roll); 
//	Para_pos.v_y  *=Sonar.UltraHeight*cos(Para_APM.Angle_pitch)*cos(Para_APM.Angle_roll); 
				
}
void Get_Flow_Prepare(UINT_8 data){
	static u8 RxBuffer[50];
	static u8 _data_len = 0,_data_cnt = 0;
	static u8 state = 0;
	if(state==0&&data==0xFA)
	{
		state=1;
		RxBuffer[0]=data;
	}
	else if(state==1&&data==0x01)
	{
		state=2;
		RxBuffer[1]=data;
	}
	else if(state==2&&data==0X01)
	{
		state=3;
		RxBuffer[2]=data;
	}
	else if(state==3&&data==Flow_Len)
	{
		state = 4;
		RxBuffer[3]=data;
		_data_len = data;
		_data_cnt = 0;
	}
	else if(state==4&&_data_len>0)
	{
		RxBuffer[4+_data_cnt++]=data;
		if(_data_len==_data_cnt+4 )
			state = 5;
	}
	else if(state==5)
	{
		state = 0;
		RxBuffer[4+_data_cnt]=data;
	if( RxBuffer[_data_len-1] == (RxBuffer[0]&RxBuffer[1]&RxBuffer[2]&RxBuffer[3]) + 1)
		Get_Flow_Data(RxBuffer+4,_data_cnt);
	}
	else
		state = 0;
}