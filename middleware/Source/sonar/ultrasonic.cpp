#include "include.h"
uint32_t ultra_count = 0;
float distance = 0;
int Ultra_Dis[5];
int Ultra_OK=1;
UltraConf Sonar;
/*************************************************************************************************
@f_name: void Ultra_INIT(u16 Prescaler,u16 Period)
@brief:
@param:	 Prescaler    Period
@return: None
***************************************************************************************************/
void UltraConf::TIM2_Init(uint32_t TIM_scale, uint32_t TIM_Period)
{
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
//    NVIC_InitTypeDef  NVIC_InitStructure;
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

    TIM_TimeBaseStructure.TIM_Period = TIM_Period;//
    TIM_TimeBaseStructure.TIM_Prescaler = TIM_scale;
    TIM_TimeBaseStructure.TIM_ClockDivision = 0;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);
    TIM_Cmd(TIM2, ENABLE);
}

/********************************************************************************************************************************************
@f_name: void Ultran_Init(void)
@brief:
@param:	 None
@return: None
*********************************************************************************************************************************************/
bool UltraConf::Ultran_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    EXTI_InitTypeDef EXTI_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    RCC_APB2PeriphClockCmd(ULTRA_TRIGCLK, ENABLE);
    RCC_APB2PeriphClockCmd(ULTRA_ECHOCLK, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);

    GPIO_InitStructure.GPIO_Pin = ULTRA_TRIG;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;  //10MHz
    GPIO_Init(ULTRA_TRIGPORT, &GPIO_InitStructure);
    GPIO_ResetBits(ULTRA_TRIGPORT,ULTRA_TRIG);

    GPIO_InitStructure.GPIO_Pin = ULTRA_ECHO;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;//下拉输入
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;//10M
    GPIO_Init(ULTRA_ECHOPORT, &GPIO_InitStructure);//GPIOE7,8

		EXTI_ClearITPendingBit(EXTI_Line8);  
    GPIO_EXTILineConfig(GPIO_PortSourceGPIOE,GPIO_PinSource8);//PE8  为GPIOE的PIN8  

    /* EXTI_Line8 */
    EXTI_InitStructure.EXTI_Line = EXTI_Line8;//LINE8
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;//
    EXTI_Init(&EXTI_InitStructure);

    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);

    NVIC_InitStructure.NVIC_IRQChannel=EXTI9_5_IRQn ;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority=1;
    NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    TIM2_Init(72-1,0xffffffff);
		
		return true;
}

/*************************************************************************************************
@f_name: void Ranging(void)
@brief:
@param:	 None
@return: None
***************************************************************************************************/

void UltraConf::Ultra_Ranging(void)
{
    GPIO_SetBits(ULTRA_TRIGPORT, ULTRA_TRIG);
    delay_us(25);
    GPIO_ResetBits(ULTRA_TRIGPORT, ULTRA_TRIG);
}

/*************************************************************************************************
@f_name: void EXTI3_IRQHandler(void)
@brief:
@param:	 None
@return: None
***************************************************************************************************/
/* sonar config*/
float sonar_distance_filtered = 0.0f; // distance in meter
float sonar_distance_raw = 0.0f; // distance in meter
bool distance_valid = false;

extern uint32_t get_boot_time_us(void);
static volatile uint32_t last_measure_time = 0;
static volatile uint32_t measure_time = 0;
static volatile float dt = 0.0f;
static volatile int valid_data;
static volatile int data_counter = 0;
static volatile int data_valid = 0;
static volatile int new_value = 0;

static volatile uint32_t sonar_measure_time_interrupt = 0;
static volatile uint32_t sonar_measure_time = 0;

/* kalman filter states */
float x_pred = 0.0f; // m
float v_pred = 0.0f;
float x_post = 0.0f; // m
float v_post = 0.0f; // m/s

float sonar_raw = 0.0f;  // m

bool sonar_valid = false;

void EXTI9_5_IRQHandler(void)
{

   if(EXTI_GetFlagStatus(EXTI_Line8))  //
   {

     if(GPIO_ReadInputDataBit(ULTRA_ECHOPORT,ULTRA_ECHO)==SET)  //
      {
         Ultra_OK = 1;
         TIM_SetCounter(TIM2,0);
         TIM_Cmd(TIM2, ENABLE);    //
      }

      if(GPIO_ReadInputDataBit(ULTRA_ECHOPORT,ULTRA_ECHO)==RESET)  //
      {
         TIM_Cmd(TIM2, DISABLE);    //
         ultra_count=TIM_GetCounter(TIM2);
         /* get sonar data */
         Sonar.Ultra_Distance();


      }
      EXTI_ClearFlag(EXTI_Line8);	   //
   }
}

/*************************************************************************************************
@f_name: void Ultra_Distance(void)
@brief:
@param:	 None
@return: None
***************************************************************************************************/

void UltraConf::Ultra_Distance(void)
{

        distance=173.641*((double)ultra_count/1000000.0f);//m

//        /* use real-world maximum ranges to cut off pure noise */
//        if ((distance > SONAR_MIN*SONAR_SCALE) && (distance < SONAR_MAX*SONAR_SCALE))
//        {
//            /* it is in normal sensor range, take it */
//            last_measure_time = measure_time;
//            measure_time = get_sonar_measure_time();
//            sonar_measure_time_interrupt = measure_time;
//            dt = ((float)(measure_time - last_measure_time)) / 1000.0f;
//            valid_data = distance;
//            new_value = 1;
//            sonar_valid = true;
//        }
}
//****************************关于卡尔曼滤波的讲解***********************/
//假设你有两个传感器，测的是同一个信号。
//可是它们每次的读数都不太一样，
//怎么办？取平均。
//再假设你知道其中贵的那个传感器应该准一些，
//便宜的那个应该差一些。
//那有比取平均更好的办法吗？
//加权平均。怎么加权？
//假设两个传感器的误差都符合正态分布，
//假设你知道这两个正态分布的方差，
//用这两个方差值，
//（此处省略若干数学公式），
//你可以得到一个“最优”的权重。
//接下来，重点来了：假设你只有一个传感器，
//但是你还有一个数学模型。模型可以帮你算出一个值，
//但也不是那么准。怎么办？把模型算出来的值，和传感器测出的值，
//（就像两个传感器那样），取加权平均。OK，最后一点说明：你的模型其实只是一个步长，
//也就是说，知道x(k)，我可以求x(k+1)。
//问题是x(k)是多少呢？
//答案：x(k)就是你上一步卡尔曼滤波得到的、所谓加权平均之后的那个、对x在k时刻的最佳估计值。
//于是迭代也有了。这就是卡尔曼滤波。（无公式）
/*************************************************************************/

float average_sonar[5];
void UltraConf::sonar_filter()
{
//    /* no data for long time */
//    if (dt > 0.25f) // more than 2 values lost
//    {
//        v_pred = 0;
//    }  
//	   x_pred = x_post + dt * v_post;
//    v_pred = v_post;
//		float x_new = distance/100;  //cm转换成m
//		sonar_raw = x_new;
//    x_post = x_pred + SONAR_KALMAN_L1 * (x_new - x_pred);
//    v_post = v_pred + SONAR_KALMAN_L2 * (x_new - x_pred);
		
		
  

	
	float sum = 0;
	average_sonar[0]=distance*100;//m->cm  注意上位机显示的数据放大了100倍
	sum+=average_sonar[0];
	for(int i=1;i<5;i++)
	{
		average_sonar[i]=average_sonar[i-1];
		sum+=average_sonar[i];
	}
	 x_post = sum/5;
}


/**
  * @brief  Read out newest sonar data
  *
  * @param  sonar_value_filtered Filtered return value
  * @param  sonar_value_raw Raw return value
  */
void UltraConf::sonar_read(float* sonar_value_filtered, float* sonar_value_raw)
{
	sonar_filter();
	*sonar_value_raw=average_sonar[0];
	*sonar_value_filtered = x_post;
//    sonar_valid = true;

//    /* getting new data with only around 10Hz */
//    if (new_value) {
//        sonar_filter();
//        new_value = 0;
//    }

//    /* catch post-filter out of band values */
//    if (x_post < SONAR_MIN || x_post > SONAR_MAX) {
//        sonar_valid = false;
//    }

//    *sonar_value_filtered = x_post;
//    *sonar_value_raw = sonar_raw;


}

uint32_t get_sonar_measure_time()
{
    return Sys_Time.Get_Cycle_T(0);
}

