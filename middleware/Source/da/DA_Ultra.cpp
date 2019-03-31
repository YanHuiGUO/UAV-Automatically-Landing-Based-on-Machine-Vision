#include "DA_Ultra.h"
UltraConf Soner ;
//UltraConf���캯��
UltraConf::UltraConf()
{
	for(int i=0;i<UltraBufferLen;i++)
	{
		Ultra_TX_Arr[i] =0;
		Ultra_RX_Arr[i] =0;
	}
	UltraHeight = 0;
	ultra_cnt = 0;
	Ultra_Ok  = 0;
}


//��������ʼ��
int UltraConf::Ultra_INIT(int UltraType)
{
		//DAC_CH1_INIT();
		switch(UltraType)
		{
			case US100:
									//Ultra.Usart_Init(Ultra_USART,Ultra_USART_Baud);
									//Ultra_TX_Arr[0] = 0x55;
									break;
			case KS103:break;
			case HR_04:break;
		}
	return 1;
}
////PA4 DAC_Channal_1��� 
void UltraConf::DAC_CH1_INIT(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	DAC_InitTypeDef DAC_InitType;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE );	  
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_DAC, ENABLE );	   

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;				
 	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 		//���ù̶���ѹ���ʱ��Ҫ��������� 
 	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
 	GPIO_Init(GPIOA, &GPIO_InitStructure);
	GPIO_SetBits(GPIOA,GPIO_Pin_4)	;//PA.4 
					
	DAC_InitType.DAC_Trigger=DAC_Trigger_None;	//��ʹ�ô���Դ
	DAC_InitType.DAC_WaveGeneration=DAC_WaveGeneration_None;//�����ò�����״
	//DAC_InitType.DAC_LFSRUnmask_TriangleAmplitude=DAC_TriangleAmplitude_4095;//���÷�ֵ�޶�
	DAC_InitType.DAC_OutputBuffer=DAC_OutputBuffer_Enable ;	//������������ã���ǿ�������
	DAC_Init(DAC_Channel_1,&DAC_InitType);	 //

	
	DAC_DMACmd(DAC_Channel_1, DISABLE);		//�ص�DMA
	DAC_Cmd(DAC_Channel_1, ENABLE);  //ʹ��DAC
  
  DAC_SetChannel1Data(DAC_Align_12b_R, 0);  //12�ֽ�����룬��ʼ���0V
}

//���ڳ�����

void UltraConf::Ultra_TX(int UltraType)
{
			switch(UltraType)
		{
			case US100:
									Ultra.Ultra_Send(Ultra_TX_Arr[0]);
									break;
			case KS103: break;
			case HR_04: break;
		}
	
}

//��ȡ���������벢ת��Ϊ DA
UINT_16 UltraConf::GetUltra(const UINT_8 buffer)
{
	Ultra_RX_Arr[ultra_cnt]	= buffer;
	ultra_cnt ++;
	if(ultra_cnt == 2)
		{
			ultra_cnt = 0;	
			float UltraTemp = (int)(Ultra_RX_Arr[0]<<8)+Ultra_RX_Arr[1];
			UltraTemp = UltraTemp/10;
			UltraHeight	=	Moving_Average(0,10,UltraTemp);
			Ultra_Ok = 0;
		}
	return UltraHeight ;
}

//��ǯλ������ǯס��ѹ������3.3V
const float UltraVperM_Offset = 0 *1000 ; //0
const float UltraVperM = ((float)(3300-UltraVperM_Offset)/350);//3.3v
void UltraConf::Dac1_Set_Vol(void)
{
	float vol =0;
	float Voltage = 3.3;
	if(Ultra_Ok == 1)
	{
		Ultra_Ok = 0;
		vol = UltraHeight*UltraVperM;
		vol/=1000;
		vol=vol*4096/Voltage;
		
		vol = vol>4096? 4095:vol;//�޷�
		
		DAC_SetChannel1Data(DAC_Align_12b_R,vol);//12λ�Ҷ���
		DAC_SoftwareTriggerCmd(DAC_Channel_1,ENABLE);
	}
}


#define WIDTH_NUM 20
#define FIL_ITEM  5
float filter_tmp[FIL_ITEM][WIDTH_NUM ];
float filter_out[FIL_ITEM];
u8 fil_cnt[FIL_ITEM],fil_cnt_old[FIL_ITEM];
//������ֵ�˲�����
float Moving_Average(u8 item,u8 width_num,float in)
{
	if(item >= FIL_ITEM || width_num >= WIDTH_NUM )
	{
		return 0;
	}
	else
	{
		if( ++fil_cnt[item] > width_num )	
		{
			fil_cnt[item] = 0;
			fil_cnt_old[item] = 1;
		}
		else
		{
			fil_cnt_old[item] = (fil_cnt[item] == width_num)? 0 : (fil_cnt[item] + 1);
		}
		
		filter_tmp[item][ fil_cnt[item] ] = in;
		filter_out[item] += ( in - ( filter_tmp[item][ fil_cnt_old[item] ]  ) )/(float)( width_num ) ;//+ 0.01 *filter_out[item]
		
		return ( filter_out[item] );
	}

}
