#include "include.h"
//串口类实例化
Usart apmMAV;
Usart Flow;
Usart AnalysisUsart;
Usart ReserveUsart;
para_group Para_APM;
para_vision Para_pos;
/// 重定向c库函数printf到USART1
extern "C" int fputc(int ch, FILE *f)
{
		/* 发送一个字节数据到USART1 */
		USART_SendData(USART1, (uint8_t) ch);
		
		/* 等待发送完毕 */
		while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);		
	
		return (ch);
}

/// 重定向c库函数scanf到USART1
extern "C"  int fgetc(FILE *f)
{
		/* 等待串口1输入数据 */
		while (USART_GetFlagStatus(USART1, USART_FLAG_RXNE) == RESET);

		return (int)USART_ReceiveData(USART1);
}
 
float constrain_float(float amt, float low, float high)
{
	if(amt-low<0.000001f) return low;
	else if(amt-high>0.000001f) return high;
	return amt;
}


//缓冲数组
//UINT_8 PC_SEND2_APM[APMLEN]  = {0};
//UINT_8 APM_SEND2_PC[APMLEN]  = {0};
//UINT_8 PC_SEND2_APM_TEMP[APMLEN]  = {0};
//UINT_8 APM_SEND2_PC_TEMP[APMLEN]  = {0};

UINT_8 SEND2_APM[APMLEN]  = {0};
UINT_8 GET_APM[APMLEN]  = {0};
UINT_8 From_PC[PC_LEN] = {0};
UINT_8 SEND2_PC[PC_LEN] = {0};

///图像速度方向，和加速度方向是同向的，以APM壳子上的箭头为正
//      x(+)
//			|
//			|
//	----|----->y(+)
//			|


filter_para filter_vx;
filter_para filter_vy;
double ProcessNiose_Q_x=3,ProcessNiose_Q_y=3;
double MeasureNoise_R_x=60,MeasureNoise_R_y=60;
double InitialPrediction = 0.5;
double KalmanFilter_vx(const double ResrcData)
{
        double R = MeasureNoise_R_x;
        double Q = ProcessNiose_Q_x;
        static double x_last;
        double x_mid = x_last;
        double x_now;
        static double p_last;
        double p_mid ;
        double p_now;
        double kg;        
        x_mid=x_last; //x_last=x(k-1|k-1),x_mid=x(k|k-1)
        p_mid=p_last+Q; //p_mid=p(k|k-1),p_last=p(k-1|k-1),Q=??
        kg=p_mid/(p_mid+R); //kg?kalman filter,R???
        x_now=x_mid+kg*(ResrcData-x_mid);//??????? 
        p_now=(1-kg)*p_mid;//??????covariance        
        p_last = p_now; //??covariance?
        x_last = x_now; //???????
        return x_now;                
}
double KalmanFilter_vy(const double ResrcData)
{
        double R = MeasureNoise_R_y;
        double Q = ProcessNiose_Q_y;
        static double x_last;
        double x_mid = x_last;
        double x_now;
        static double p_last;
        double p_mid ;
        double p_now;
        double kg;        
        x_mid=x_last; //x_last=x(k-1|k-1),x_mid=x(k|k-1)
        p_mid=p_last+Q; //p_mid=p(k|k-1),p_last=p(k-1|k-1),Q=??
        kg=p_mid/(p_mid+R); //kg?kalman filter,R???
        x_now=x_mid+kg*(ResrcData-x_mid);//??????? 
        p_now=(1-kg)*p_mid;//??????covariance        
        p_last = p_now; //??covariance?
        x_last = x_now; //???????
        return x_now;                
}
double ProcessNiose_Q_pos_x=3,ProcessNiose_Q_pos_y=3;
double MeasureNoise_R_pos_x=60,MeasureNoise_R_pos_y=60;
double InitialPrediction_pos = 0.5;
double KalmanFilter_pos_x(const double ResrcData)
{
        double R = MeasureNoise_R_pos_x;
        double Q = ProcessNiose_Q_pos_x;
        static double x_last ;
        double x_mid = x_last;
        double x_now;
        static double p_last;
        double p_mid ;
        double p_now;
        double kg;        
        x_mid=x_last; //x_last=x(k-1|k-1),x_mid=x(k|k-1)
        p_mid=p_last+Q; //p_mid=p(k|k-1),p_last=p(k-1|k-1),Q=??
        kg=p_mid/(p_mid+R); //kg?kalman filter,R???
        x_now=x_mid+kg*(ResrcData-x_mid);//??????? 
        p_now=(1-kg)*p_mid;//??????covariance        
        p_last = p_now; //??covariance?
        x_last = x_now; //???????
        return x_now;                
}
double KalmanFilter_pos_y(const double ResrcData)
{
        double R = MeasureNoise_R_pos_y;
        double Q = ProcessNiose_Q_pos_y;
        static double x_last;
        double x_mid = x_last;
        double x_now;
        static double p_last;
        double p_mid ;
        double p_now;
        double kg;        
        x_mid=x_last; //x_last=x(k-1|k-1),x_mid=x(k|k-1)
        p_mid=p_last+Q; //p_mid=p(k|k-1),p_last=p(k-1|k-1),Q=??
        kg=p_mid/(p_mid+R); //kg?kalman filter,R???
        x_now=x_mid+kg*(ResrcData-x_mid);//??????? 
        p_now=(1-kg)*p_mid;//??????covariance        
        p_last = p_now; //??covariance?
        x_last = x_now; //???????
        return x_now;                
}

void Kalman_Filter_Init(void)
{
	//图像参数初始化
	Para_pos.x_last=Para_pos.y_last = 0;
	Para_pos.pixTodis = 0.5;//与高度有关系，为了简便起见不如就设个0.5 cm/pix(4m高度，70度视野角，240pix图像)
	
	//x方向
	filter_vx.P[0][0] = filter_vx.P[1][1] = 1;
	filter_vx.P[0][1] = filter_vx.P[1][0] = 0;
	filter_vx.Pdot[0] = filter_vx.Pdot[1] =filter_vx.Pdot[2]=filter_vx.Pdot[3] =0;
	filter_vx.Q_v =0.335; //0.35;
	filter_vx.Q_acc =0.5; //0.05;
	filter_vx.R_angle = 0.5;
	filter_vx.C_0 = 0.1;
	filter_vx.v_acc=filter_vx.v_err = 0;
	
	
	Para_Flow.k_comp_x = 0.285;//0.265
	
	//y方向
	filter_vy.P[0][0] = filter_vy.P[1][1] = 1;
	filter_vy.P[0][1] = filter_vy.P[1][0] = 0;
	filter_vy.Pdot[0] = filter_vy.Pdot[1] =filter_vy.Pdot[2]=filter_vy.Pdot[3] =0;
	filter_vy.Q_v =0.335; //0.35;
	filter_vy.Q_acc = 0.5;//0.05;
	filter_vy.R_angle = 0.5;
	filter_vy.C_0 = 0.1;
	filter_vy.v_acc=filter_vy.v_err = 0;
	
	Para_Flow.k_comp_y= 0.285; //0.265
}

void Kalman_Filter(float vx, float acc_x,float vy,float acc_y,float dt)
{
	//对x方向进行卡尔曼融合
		filter_vx.v_acc+=(acc_x-filter_vx.q_bias) * dt;
		filter_vx.v_err = vx - filter_vx.v_acc;
		filter_vx.Pdot[0]=filter_vx.Q_v - filter_vx.P[0][1] - filter_vx.P[1][0];
		filter_vx.Pdot[1]=- filter_vx.P[1][1];
		filter_vx.Pdot[2]=- filter_vx.P[1][1];
		filter_vx.Pdot[3]=filter_vx.Q_acc;
		filter_vx.P[0][0] += filter_vx.Pdot[0] * dt;
		filter_vx.P[0][1] += filter_vx.Pdot[1] * dt;
		filter_vx.P[1][0] += filter_vx.Pdot[2] * dt;
		filter_vx.P[1][1] += filter_vx.Pdot[3] * dt;
		filter_vx.PCt_0 = filter_vx.C_0 * filter_vx.P[0][0];
		filter_vx.PCt_1 = filter_vx.C_0 * filter_vx.P[1][0];
		filter_vx.E = filter_vx.R_angle + filter_vx.C_0 * filter_vx.PCt_0;
		filter_vx.K_0 = filter_vx.PCt_0 / filter_vx.E;
		filter_vx.K_1 = filter_vx.PCt_1 / filter_vx.E;
		filter_vx.t_0 = filter_vx.PCt_0;
		filter_vx.t_1 = filter_vx.C_0 * filter_vx.P[0][1];
		filter_vx.P[0][0] -= filter_vx.K_0 * filter_vx.t_0;
		filter_vx.P[0][1] -= filter_vx.K_0 * filter_vx.t_1;
		filter_vx.P[1][0] -= filter_vx.K_1 * filter_vx.t_0;
		filter_vx.P[1][1] -= filter_vx.K_1 * filter_vx.t_1;
		filter_vx.v_acc += filter_vx.K_0 * filter_vx.v_err; 
		filter_vx.q_bias += filter_vx.K_1 * filter_vx.v_err;
		Para_pos.filter_vx = filter_vx.v_acc;
		
	//对y方向进行卡尔曼融合
		filter_vy.v_acc+=(acc_y-filter_vy.q_bias) * dt;
		filter_vy.v_err = vy - filter_vy.v_acc;
		filter_vy.Pdot[0]=filter_vy.Q_v - filter_vy.P[0][1] - filter_vy.P[1][0];
		filter_vy.Pdot[1]=- filter_vy.P[1][1];
		filter_vy.Pdot[2]=- filter_vy.P[1][1];
		filter_vy.Pdot[3]=filter_vy.Q_acc;
		filter_vy.P[0][0] += filter_vy.Pdot[0] * dt;
		filter_vy.P[0][1] += filter_vy.Pdot[1] * dt;
		filter_vy.P[1][0] += filter_vy.Pdot[2] * dt;
		filter_vy.P[1][1] += filter_vy.Pdot[3] * dt;
		filter_vy.PCt_0 = filter_vy.C_0 * filter_vy.P[0][0];
		filter_vy.PCt_1 = filter_vy.C_0 * filter_vy.P[1][0];
		filter_vy.E = filter_vy.R_angle + filter_vy.C_0 * filter_vy.PCt_0;
		filter_vy.K_0 = filter_vy.PCt_0 / filter_vy.E;
		filter_vy.K_1 = filter_vy.PCt_1 / filter_vy.E;
		filter_vy.t_0 = filter_vy.PCt_0;
		filter_vy.t_1 = filter_vy.C_0 * filter_vy.P[0][1];
		filter_vy.P[0][0] -= filter_vy.K_0 * filter_vy.t_0;
		filter_vy.P[0][1] -= filter_vy.K_0 * filter_vy.t_1;
		filter_vy.P[1][0] -= filter_vy.K_1 * filter_vy.t_0;
		filter_vy.P[1][1] -= filter_vy.K_1 * filter_vy.t_1;
		filter_vy.v_acc += filter_vy.K_0 * filter_vy.v_err; 
		filter_vy.q_bias += filter_vy.K_1 * filter_vy.v_err;
		Para_pos.filter_vy = filter_vy.v_acc;
		
}

//定点的速度与加速度融合函数
static float lpf_vx = 0,lpf_vy=0;
float k_camera_Groy_x = 2;
float k_camera_Groy_y = 2;
void Usart::Target_V_Get(float pos_x_out, float pos_y_out, float dt)
{

	//像素cm/s
	Para_pos.v_x = Para_pos.v_x * Para_pos.pixTodis;
	Para_pos.v_y = Para_pos.v_y * Para_pos.pixTodis;

	//m/s/s确保和光流的量纲一直
	float ax = Para_APM.ACC_x *100;
	float ay = Para_APM.ACC_y *100;

	
	//图像的y和加计的x同向，x和加计的y同向
	//Kalman_Filter(Para_pos.v_x,ax,Para_pos.v_y,ay,0.01);
	
//	Para_pos.filter_vx = Para_pos.v_x;
//	Para_pos.filter_vy = Para_pos.v_y;
//下面是之前融合图像数据的
//	Kalman_Filter(Para_pos.v_x,ax,Para_pos.v_y,ay,0.01);
	
//下面是一阶互补滤波，效果没卡尔曼好，系数是0.35
//	Para_pos.filter_vx = Para_pos.filter_vx_coeff * Para_pos.v_x\
//		+ (1 - Para_pos.filter_vx_coeff)*(Para_pos.filter_vx + ax*dt);
//	Para_pos.filter_vy = Para_pos.filter_vy_coeff * Para_pos.v_y\
//		+ (1 - Para_pos.filter_vy_coeff)*(Para_pos.filter_vy + ay*dt);
	

	Para_pos.filter_vx=KalmanFilter_vx(Para_pos.v_x);
	
	Para_pos.filter_vy=KalmanFilter_vy(Para_pos.v_y);
	

	Para_pos.filter_vx = Para_pos.filter_vx+k_camera_Groy_x*(-Para_APM.Groy_y);
	Para_pos.filter_vy = Para_pos.filter_vy+k_camera_Groy_y*Para_APM.Groy_x;
}


void Usart::Target_VisonData_Deal(float dt)
{

	float vx_temp = Para_pos.X_offset_get-Para_pos.x_last;
	float vy_temp = Para_pos.Y_offset_get-Para_pos.y_last;
	
	
	Para_pos.v_x = vx_temp/dt;
	Para_pos.v_y = vy_temp/dt;
	
	Para_pos.x_last = Para_pos.X_offset_get;
	Para_pos.y_last = Para_pos.Y_offset_get;
	
	//利用光流算速度，与解算的图像的定义的xy方向是反的
	//先暂时不做和超声波的融合
	//用固定值代表高度
//	float vx_temp = Para_Flow.flow_comp_x*2;
//	float vy_temp = Para_Flow.flow_comp_y*2;
	
	
//	Para_pos.v_x = vx_temp;
//	Para_pos.v_y = vy_temp;
//	


}


void Usart::Float2Byte(float floatNum, unsigned char* byteArry)
{
	char* pchar = (char*)&floatNum;
	for (int i = 0; i<sizeof(float); i++)
	{
		*byteArry = *pchar;
		pchar++;
		byteArry++;
	}
}

float Usart::Byte2Float(unsigned char* byteArry)
{
	return *((float*)byteArry);
}


void Usart::generatePacket_ToAPM(unsigned char* byteArry,int len)
{
	UINT_8 Sonar_Bytes[4]={0};
	UINT_8 X_Excursion[4]={0};
	UINT_8 Y_Excursion[4]={0};
	UINT_8 Direction[4]={0};
	UINT_8 echo[4]={0};
	UINT_8 Estimate_Dis[4]={0};
	UINT_8 FLAG_RED_BLUE[4]={0};
	int index = 0;
	if(len<=APMLEN)
	{
			Float2Byte(Sonar.UltraHeight,Sonar_Bytes);
			Float2Byte(Para_pos.X_offset_get,X_Excursion);
			Float2Byte(Para_pos.Y_offset_get,Y_Excursion);
			Float2Byte(Para_pos.Dir_get,Direction);
			Float2Byte(Para_pos.e_get,echo);	
			Float2Byte(Para_pos.Dis_ElimateByIMG,Estimate_Dis);//这里改成高度估计
			Float2Byte((float)(Para_pos.is_GetRed<<1|Para_pos.is_GetBlue),FLAG_RED_BLUE);//这里改成红蓝圆的标志位
		
			byteArry[index]=0xFA;index++;
			byteArry[index]=0x01;index++;
			byteArry[index]=0x01;index++;
			byteArry[index]=Frame_Len;index++;
			int i=0;
			for( i=index;i<index+4;i++)
				byteArry[i]=Sonar_Bytes[i-index];
			index = i;
		
			for( i=index;i<index+4;i++)
				byteArry[i]=X_Excursion[i-index];
			index = i;
		
			for( i=index;i<index+4;i++)
				byteArry[i]=Y_Excursion[i-index];
			index = i;
			
			for( i=index;i<index+4;i++)
				byteArry[i]=Direction[i-index];
			index = i;
			
			for( i=index;i<index+4;i++)
				byteArry[i]=echo[i-index];
			index = i;
		
			for( i=index;i<index+4;i++)
				byteArry[i]=Estimate_Dis[i-index];
			index = i;
			
			for( i=index;i<index+4;i++)
				byteArry[i]=FLAG_RED_BLUE[i-index];
			index = i;
		
			byteArry[index] = (byteArry[0]&byteArry[1]&byteArry[2]&byteArry[3])+1;
	}
}
//***************************************************//
//串口引脚分配
//PA9，PA10 USART1 链接手机蓝牙
//PB10,PB11 USART3 链接APM的串口
//PA2，PA3  USART2 链接PC波形上位机的串口
//PC10，PC11 USART4 链接光流的串口
//***************************************************//
int Usart::Usart_Init(int USART,unsigned int Baud)
{

	//private init
	apmStr = 0;

	//usart config
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure; 
	DMA_InitTypeDef DMA_InitStructure;
	
	//***********************************************************
	//注意，有两个接收的DMA中断根本没有使用,但是使用了DMA接受数据，
	//			见中断函数中
	//***********************************************************
	switch(USART)
	{
		case APM_USART:			//32连接数传的口，传回信息给PC，转接PC信息到内存
			
					/* config USART1 clock */
					RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
					RCC_APB2PeriphClockCmd((RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO), ENABLE);
					
					/* USART1 GPIO config */
					/* Configure USART1 Tx (PA.09) as alternate function push-pull */
					GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
					GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
					GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
					GPIO_Init(GPIOA, &GPIO_InitStructure);
				
					/* Configure USART1 Rx (PA.10) as input floating */
					GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
					GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
					GPIO_Init(GPIOA, &GPIO_InitStructure);
						
		
					/* USART1 mode config */
					USART_InitStructure.USART_BaudRate = Baud;
					USART_InitStructure.USART_WordLength = USART_WordLength_8b;
					USART_InitStructure.USART_StopBits = USART_StopBits_1;
					USART_InitStructure.USART_Parity = USART_Parity_No ;
					USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
					USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
					USART_Init(USART1, &USART_InitStructure); 
				

					//配置TX DMA功能
					RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE); 
					NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel4_IRQn;   
					NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;     
					NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
					NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
					NVIC_Init(&NVIC_InitStructure);

					DMA_Cmd(DMA1_Channel4, DISABLE);                          
					DMA_DeInit(DMA1_Channel4);                                 
					DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)(&USART1->DR);
					DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)SEND2_PC;         
					DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;                     
					DMA_InitStructure.DMA_BufferSize = PC_BUFFER_LEN;                   
					DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;       
					DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;                
					DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte; 
					DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;         
					DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;                         
					DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;                 
					DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;                            
					DMA_Init(DMA1_Channel4, &DMA_InitStructure);               
														 
					DMA_Cmd(DMA1_Channel4, DISABLE); 
					DMA_ITConfig(DMA1_Channel4, DMA_IT_TC, ENABLE);            
					USART_DMACmd(USART1,USART_DMAReq_Tx,ENABLE);      
								
					//----------------------------------------------------------------
					//配置RX DMA功能
					DMA_Cmd(DMA1_Channel5, DISABLE);                         
					DMA_DeInit(DMA1_Channel5);                                 
					DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)(&USART1->DR);
					DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)From_PC;         
					DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;                      
					DMA_InitStructure.DMA_BufferSize = PC_BUFFER_LEN;                     
					DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;       
					DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;                 
					DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte; 
					DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;         
					DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;                           
					DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;                 
					DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;                           
					DMA_Init(DMA1_Channel5, &DMA_InitStructure);              
					DMA_ClearFlag(DMA1_FLAG_GL5);
					
					USART_DMACmd(USART1,USART_DMAReq_Rx,ENABLE);   
					DMA_Cmd(DMA1_Channel5, ENABLE); 		
					
					USART_ITConfig(USART1,USART_IT_TC,DISABLE);  
					USART_ITConfig(USART1,USART_IT_RXNE,DISABLE);  
					USART_ITConfig(USART1,USART_IT_IDLE,ENABLE);
					USART_Cmd(USART1, ENABLE);

				
					/* Enable the USARTy Interrupt */
					NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;	 
					NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
					NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
					NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
					NVIC_Init(&NVIC_InitStructure);
					
					
					mavMsgFlag.Flag_sendToPC = 0;
					break;
		
		case Analysis_USART:			//与APM交换数据的串口
					/* config USART3 clock */
					RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);
					RCC_APB2PeriphClockCmd((RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO), ENABLE);
					
					/* USART3 GPIO config */
					/* Configure USART3 Tx (PB.10) as alternate function push-pull */
					GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
					GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
					GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
					GPIO_Init(GPIOB, &GPIO_InitStructure);
				
					/* Configure USART3 Rx (PB.11) as input floating */
					GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
					GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
					GPIO_Init(GPIOB, &GPIO_InitStructure);

					/* USART3 mode config */
					USART_InitStructure.USART_BaudRate = Baud;
					USART_InitStructure.USART_WordLength = USART_WordLength_8b;
					USART_InitStructure.USART_StopBits = USART_StopBits_1;
					USART_InitStructure.USART_Parity = USART_Parity_No ;
					USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
					USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
					USART_Init(USART3, &USART_InitStructure); 
					
					RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE); 
					NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel2_IRQn;   
					NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;     
					NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;
					NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
					NVIC_Init(&NVIC_InitStructure);
					
					//
					//配置TX DMA功能
					DMA_Cmd(DMA1_Channel2, DISABLE);                           
					DMA_DeInit(DMA1_Channel2);                                 
					DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)(&USART3->DR);
					DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)SEND2_APM;         
					DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;                      
					DMA_InitStructure.DMA_BufferSize = APMLEN;                     
					DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;        
					DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;                 
					DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte; 
					DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;         
					DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;                           
					DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;                 
					DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;                            
					DMA_Init(DMA1_Channel2, &DMA_InitStructure);              
														
					DMA_Cmd(DMA1_Channel2, DISABLE); 
					DMA_ITConfig(DMA1_Channel2, DMA_IT_TC, ENABLE);           
					USART_DMACmd(USART3,USART_DMAReq_Tx,ENABLE); 	
					
					
//					//----------------------------------------------------------------
//					//配置RX DMA功能
					DMA_Cmd(DMA1_Channel3, DISABLE);                          
					DMA_DeInit(DMA1_Channel3);                                 
					DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)(&USART3->DR);
					DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)GET_APM;         
					DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;                      
					DMA_InitStructure.DMA_BufferSize = APMLEN;                     
					DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;        
					DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;                 
					DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte; 
					DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;         
					DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;                           
					DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;                 
					DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;                            
					DMA_Init(DMA1_Channel3, &DMA_InitStructure);              
					DMA_ClearFlag(DMA1_FLAG_GL3);
					
					DMA_ITConfig(DMA1_Channel3, DMA_IT_TC, ENABLE);      
					USART_DMACmd(USART3,USART_DMAReq_Rx,ENABLE);   
					DMA_Cmd(DMA1_Channel3, ENABLE); 
//					
					USART_ITConfig(USART3,USART_IT_TC,DISABLE);  
					USART_ITConfig(USART3,USART_IT_RXNE,DISABLE);  
					
					USART_ITConfig(USART3,USART_IT_IDLE,ENABLE);

					NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;	 
					NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
					NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
					NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
					NVIC_Init(&NVIC_InitStructure);

					/* Enable the USARTy Interrupt */
					USART_Cmd(USART3, ENABLE);				
					//				
					mavMsgFlag.Flag_sendToAPM = 0;
					break;
					
		case Reserve_USART:			//预留串口
						/* config USART2 clock */
					RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
					RCC_APB2PeriphClockCmd((RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO), ENABLE);
					
					/* USART2 GPIO config */
					/* Configure USART2 Tx (PA.02) as alternate function push-pull */
					GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
					GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
					GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
					GPIO_Init(GPIOA, &GPIO_InitStructure);
				
					/* Configure USART2 Rx (PA.03) as input floating */
					GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
					GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
					GPIO_Init(GPIOA, &GPIO_InitStructure);
						
					/* USART2 mode config */
					USART_InitStructure.USART_BaudRate = Baud;
					USART_InitStructure.USART_WordLength = USART_WordLength_8b;
					USART_InitStructure.USART_StopBits = USART_StopBits_1;
					USART_InitStructure.USART_Parity = USART_Parity_No ;
					USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
					USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
					USART_Init(USART2, &USART_InitStructure); 
					USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);
					USART_Cmd(USART2, ENABLE);
					USART_ClearFlag(USART2, USART_FLAG_TC);
					
					/* Enable the USARTy Interrupt */
					NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;	 
					NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
					NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
					NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
					NVIC_Init(&NVIC_InitStructure);
					break;
					
	 case Flow_USART:	 //光流数据串口

					
					RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART4, ENABLE);
					RCC_APB2PeriphClockCmd((RCC_APB2Periph_GPIOC | RCC_APB2Periph_AFIO), ENABLE);
					
					/* USART2 GPIO config */
					/* Configure USART4 Tx (PC.10) as alternate function push-pull */
					GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
					GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
					GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
					GPIO_Init(GPIOC, &GPIO_InitStructure);
				
					/* Configure USART4 Rx (PC.11) as input floating */
					GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
					GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
					GPIO_Init(GPIOC, &GPIO_InitStructure);
						
					/* USART4 mode config */
					USART_InitStructure.USART_BaudRate = Baud;
					USART_InitStructure.USART_WordLength = USART_WordLength_8b;
					USART_InitStructure.USART_StopBits = USART_StopBits_1;
					USART_InitStructure.USART_Parity = USART_Parity_No ;
					USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
					USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
					USART_Init(UART4, &USART_InitStructure); 
					USART_ITConfig(UART4, USART_IT_RXNE, ENABLE);
					USART_Cmd(UART4, ENABLE);
					USART_ClearFlag(UART4, USART_FLAG_TC);
					
					/* Enable the USARTy Interrupt */
					NVIC_InitStructure.NVIC_IRQChannel = UART4_IRQn;	 
					NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
					NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
					NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
					NVIC_Init(&NVIC_InitStructure);

					break;
	}
	
	return 1;
}
//与apm接口串口发送函数
void Usart::apmMAV_Send(u8 buffer)
{
	/* 发送一个字节数据到USART1 */
	USART_SendData(USART1, (uint8_t) buffer);
		
	/* 等待发送完毕 */
	while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);
	
}
//与超声波接口串口发送函数
void Usart::Ultra_Send(u8 buffer)
{
	/* 发送一个字节数据到USART1 */
	USART_SendData(UART4, (uint8_t) buffer);
		
	/* 等待发送完毕 */
	while (USART_GetFlagStatus(UART4, USART_FLAG_TXE) == RESET);
	
}
//与上位机接口串口发送函数
void Usart::Analysis_Send(u8 buffer)
{
	/* 发送一个字节数据到USART2 */
	USART_SendData(USART3, (uint8_t) buffer);
		
	/* 等待发送完毕 */
	while (USART_GetFlagStatus(USART3, USART_FLAG_TXE) == RESET);
	
}
//预留接口串口发送函数
void Usart::Reserve_Send(unsigned char *DataToSend ,u8 data_num)
{
	u8 i;
	for(i=0;i<data_num;i++)
	{
		TxBuffer[_tcount++] = *(DataToSend+i);
	}
	if(!(USART2->CR1 & USART_CR1_TXEIE))
	{
		USART_ITConfig(USART2, USART_IT_TXE, ENABLE); 
	}
	
}
void AnalysePacket_APM(const uint8_t* buffer,int len)
{
	bool gethead = false;
	int recieve_start = 0;
	
	if (len <= APMLEN)
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
		if (recieve_start + ToAnalysis_Len < APMLEN)
		{
			//协议 :帧头  地址位  功能位	帧长				数据
			//			0xFA   0x01    0x01   0x19(25)  4 Bytes    ...   ....   ....    ....    头四个字节相与
			//注意帧尾一定不要把帧尾校验搞成和有效数据位为0时也同时可能出现0的情况，这样的话就会出现丢包的数据都当做有效数据
		
			if (gethead == true 
				&& buffer[recieve_start + 1] == 0x01 
				&& buffer[recieve_start + 2] == 0x01 
				&& buffer[recieve_start + 3] == ToAnalysis_Len
				&& buffer[recieve_start + ToAnalysis_Len - 1] 
				== ((buffer[recieve_start+4] & buffer[recieve_start + 5] & buffer[recieve_start + 6] 
				& buffer[recieve_start + 7])+1)
				)
			{
				AnalyseEnum step = ACC_x_Step ;
				gethead = false; 
				
				for (int i = recieve_start + 4; i < recieve_start + ToAnalysis_Len - 1 - 1; i = i + 4)
				{
					unsigned char tempdata[4] = { 0 };
					for (int j = 0; j < 4; j++)
					{
						if (i + j<APMLEN)
						tempdata[j] = buffer[i+j];
					}
					switch (step)
					{
					case ACC_x_Step :Para_APM.ACC_x= apmMAV.Byte2Float(tempdata);
														step = ACC_y_Step ;
														break;
					case ACC_y_Step :	Para_APM.ACC_y = apmMAV.Byte2Float(tempdata);
														step = ACC_z_Step ;
														break;
					case ACC_z_Step :Para_APM.ACC_z =apmMAV.Byte2Float(tempdata);
							
														step = Angle_roll_Step ; 
														break;
					case Angle_roll_Step :Para_APM.Angle_roll = apmMAV.Byte2Float(tempdata);
								
														step = Angle_pitch_Step ;
														break;
					case Angle_pitch_Step : Para_APM.Angle_pitch = apmMAV.Byte2Float(tempdata);
														step = Angle_yaw_Step ;
														break;
					case Angle_yaw_Step : 	Para_APM.Angle_yaw =apmMAV.Byte2Float(tempdata);
														step = TranBackHeight_Step ;
														break;
					case TranBackHeight_Step : Para_APM.TranBackHeight = apmMAV.Byte2Float(tempdata);
														step = Rserve_para1_Step ;
														break;
					case Rserve_para1_Step : Para_APM.Rserve_para1 =apmMAV.Byte2Float(tempdata);
														step = Groy_x_step ;
														break;
					case Groy_x_step : Para_APM.Groy_x = apmMAV.Byte2Float(tempdata);
														step = Groy_y_step ;
														break;
					case Groy_y_step : Para_APM.Groy_y = apmMAV.Byte2Float(tempdata);
														step = Rserve_para2_Step ;
														break;
					case Rserve_para2_Step : Para_APM.Rserve_para2 = apmMAV.Byte2Float(tempdata);
														step = Rserve_para3_Step ;
														break;
					case Rserve_para3_Step : Para_APM.Rserve_para3 = apmMAV.Byte2Float(tempdata);
														step = ACC_x_Step ;
														break;
					}
				}
			}
		}
	}
}
extern UINT_8 TEST_State;
extern UINT_8  getack;
//state1:捕捉帧头 state2:捕捉数据段 state3:解析包数据

enum {analysePacketState1=0,analysePacketState2,analysePacketState3};
//解析PC传过来的数据
//帧头  包长  第一个四字节（X）第二个四字节（Y） 第三个四字节（DIR）校验位   15Bytes
//0xea  0x0f 

UINT_8 DATA_PC_OK=0;

float k_camera_x = 85.0f;
float k_camera_y = 68.0f;
float camera_view = 65.0f;
void analysePacket(const uint8_t* str,int len)
{
	 int index = 0;
	 uint8_t temp[4]={0};
	 int gethead = 0;
	for(int i=0;i<len;i++)
	 {
		if(i+Packet_LEN-1 <PC_LEN)
		{
			if(str[i] == 0xea && str[i+1]==Packet_LEN)
			{
				index = i+2;
				gethead = 1;		
			}
		}
		else return;
	}
	
		if(gethead == 1&& (str[Packet_LEN - 1] == ((str[0] & str[1] & str[2] & str[3])+0xaa)))
		{
			gethead = 0;
			for(int i=0;i<4;i++)
			temp[i]= str[index+i];
			Para_pos.X_offset_get=apmMAV.Byte2Float(temp);
			
			index = index+4;
			for(int i=0;i<4;i++)
			temp[i]= str[index+i];
			Para_pos.Y_offset_get=apmMAV.Byte2Float(temp);
			
			index = index+4;
			for(int i=0;i<4;i++)
			temp[i]= str[index+i];
			Para_pos.e_get =apmMAV.Byte2Float(temp);
	
			
			index = index+4;
			for(int i=0;i<4;i++)
			temp[i]= str[index+i];
			Para_pos.Dir_get =apmMAV.Byte2Float(temp) ;
			
			
			index = index+4;
			for(int i=0;i<4;i++)
			temp[i]= str[index+i];
			Para_pos.Dis_ElimateByIMG =apmMAV.Byte2Float(temp) ;
			
			
			index = index+4;

			Para_pos.is_GetRed = str[index];
					
			Para_pos.is_GetBlue = str[index+1];
		
//无云台时的陀螺仪补偿倾角		
//		float pitch_comp = constrain_float(Para_APM.Angle_pitch, -0.349, 0.349);
//		float roll_comp = constrain_float(Para_APM.Angle_roll, -0.349, 0.349);
//			Para_pos.X_offset_get = pitch_comp >= 0 ? Para_pos.X_filter_pos - k_camera_x*fabs(pitch_comp) / camera_view * 240
//				: Para_pos.X_filter_pos+ k_camera_x*fabs(pitch_comp) / camera_view * 240;
//			Para_pos.Y_offset_get = roll_comp >= 0 ? Para_pos.Y_filter_pos + k_camera_y*fabs(roll_comp) / camera_view * 320
//				:Para_pos.Y_filter_pos - k_camera_y*fabs(roll_comp) / camera_view * 320;
//		

	
//			Para_pos.X_offset_get = Para_pos.X_offset_get+Para_APM.Groy_y*k_camera_Groy_x;
//			Para_pos.Y_offset_get = Para_pos.Y_offset_get-Para_APM.Groy_x*k_camera_Groy_y;	
	}
}	
		

