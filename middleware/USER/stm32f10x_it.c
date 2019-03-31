/**
  ******************************************************************************
  * @file    Project/STM32F10x_StdPeriph_Template/stm32f10x_it.c 
  * @author  MCD Application Team
  * @version V3.5.0
  * @date    08-April-2011
  * @brief   Main Interrupt Service Routines.
  *          This file provides template for all exceptions handler and 
  *          peripherals interrupt service routine.
  ******************************************************************************
  * @attention
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTI
  
  AL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2011 STMicroelectronics</center></h2>
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x_it.h"
#include "include.h"
/** @addtogroup STM32F10x_StdPeriph_Template
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/******************************************************************************/
/*            Cortex-M3 Processor Exceptions Handlers                         */
/******************************************************************************/

/**
  * @brief  This function handles NMI exception.
  * @param  None
  * @retval None
  */
void NMI_Handler(void)
{
}

/**
  * @brief  This function handles Hard Fault exception.
  * @param  None
  * @retval None
  */
void HardFault_Handler(void)
{
  /* Go to infinite loop when Hard Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Memory Manage exception.
  * @param  None
  * @retval None
  */
void MemManage_Handler(void)
{
  /* Go to infinite loop when Memory Manage exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Bus Fault exception.
  * @param  None
  * @retval None
  */
void BusFault_Handler(void)
{
  /* Go to infinite loop when Bus Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Usage Fault exception.
  * @param  None
  * @retval None
  */
void UsageFault_Handler(void)
{
  /* Go to infinite loop when Usage Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles SVCall exception.
  * @param  None
  * @retval None
  */
void SVC_Handler(void)
{
}

/**
  * @brief  This function handles Debug Monitor exception.
  * @param  None
  * @retval None
  */
void DebugMon_Handler(void)
{
}

/**
  * @brief  This function handles PendSVC exception.
  * @param  None
  * @retval None
  */
void PendSV_Handler(void)
{
}

/**
  * @brief  This function handles SysTick Handler.
  * @param  None
  * @retval None
  */
extern volatile UINT_32 sysTickUptime ;
void SysTick_Handler(void)
{
	sysTickUptime++;
}

/******************************************************************************/
/*                 STM32F10x Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32f10x_xx.s).                                            */
/******************************************************************************/

/*注意******************************************************************************/
//C++编程时要将中断函数extern “C”，否则无法进入中断
/******************************************************************************/
//预留的串口 串口中断函数

ARMAPI void DMA2_Channel4_5_IRQHandler(void)
{
		DMA_ClearFlag(DMA2_FLAG_TC5);  
		DMA_Cmd(DMA2_Channel5,DISABLE);
	
}
	//32(转接APM) to PC
ARMAPI void DMA1_Channel4_IRQHandler(void)
{
		DMA_ClearFlag(DMA1_FLAG_TC4);  
		DMA_Cmd(DMA1_Channel4,DISABLE);
		mavMsgFlag.Flag_sendToPC =0;
}
//	//PC to 32(未开中断)
//ARMAPI void DMA1_Channel5_IRQHandler(void)
//{
//		DMA_ClearFlag(DMA1_FLAG_TC5);  
//		DMA_Cmd(DMA1_Channel5,DISABLE);
//}

//APM to 32
//ARMAPI void DMA1_Channel3_IRQHandler(void)
//{
//		DMA_ClearFlag(DMA1_FLAG_TC3);  
//		DMA_Cmd(DMA1_Channel3,DISABLE);
//}

//32 to APM
ARMAPI void DMA1_Channel2_IRQHandler(void)
{
		DMA_ClearFlag(DMA1_FLAG_TC2);  
		DMA_Cmd(DMA1_Channel2,DISABLE);
		mavMsgFlag.Flag_sendToAPM =0;
//	
//		//判断是否进入组帧状态
//		if(1 ==	mavMsgFlag.WpCommond.S_1)
//		{
//			//已进入DMA发送中断，说明已发送完成
//			//清空组帧标志位，置位发送完成标志位	
//			
//			mavMsgFlag.WpCommond.S_1 = 0;
//		}
}
//与PC（手机）交换数据的口
UINT_8 DataFromPC[PC_LEN]={0};
ARMAPI void USART1_IRQHandler(void)
{
//		static int count = 0;
//		if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)
//	{ 	
//				USART_ClearFlag(USART1, USART_FLAG_TC);
//				DataFromPC[count] = (UINT_8)USART1->DR;
//				
//				count ++ ;
//				if(count == Packet_LEN)
//				{
//					count =0;
//					analysePacket(DataFromPC,count);
//					for(int i=0;i<PC_LEN;i++)
//					DataFromPC[i]=0;
//				}
//			
//			//	buffer = USART1->DR;
//	} 

	
	uint32_t temp = 0;
	USART_ClearFlag(USART1, USART_IT_IDLE);

//串口空闲中断
	if(USART_GetITStatus(USART1, USART_IT_IDLE) != RESET)  
	{  
			//USART_ClearFlag(USART1,USART_IT_IDLE); 
			//清中断标志位
			temp = USART1->SR;  
			temp = USART1->DR; 
		
			DMA_Cmd(DMA1_Channel5,DISABLE); 
		
			temp = PC_BUFFER_LEN - DMA_GetCurrDataCounter(DMA1_Channel5);  
							
        for (int i = 0;i < temp;i++)  
        { 			
						DataFromPC[i] = From_PC[i];  
					//	SEND2_PC[i]=DataFromPC[i];
        }
					analysePacket(DataFromPC,temp);	
				
				//清空缓冲区
				for(int i=0;i<PC_LEN;i++)
				{
					From_PC[i]=0;
					DataFromPC[i]=0;
				}
			
				DMA1_Channel5->CNDTR =PC_BUFFER_LEN;         
        DMA_Cmd(DMA1_Channel5,ENABLE);  
		
	}
			
}
u8 TxBuffer[256];
u8 TxCounter=0; 
u8 _tcount = 0;
ARMAPI void USART2_IRQHandler(void)
{
	u8 com_data;
	
	if(USART2->SR & USART_SR_ORE)//ORE中断
	{
		com_data = USART2->DR;
	}

  //接收中断
	if( USART_GetITStatus(USART2,USART_IT_RXNE) )
	{
		USART_ClearFlag(USART2, USART_FLAG_TC);

		com_data = USART2->DR;
		ANO_DT_Data_Receive_Prepare(com_data);
	}
	//发送（进入移位）中断
	if( USART_GetITStatus(USART2,USART_IT_TXE ) )
	{
		USART2->DR = TxBuffer[TxCounter++]; //写DR清除中断标志          
		if(TxCounter == _tcount)
		{
			TxCounter = _tcount=0;
			USART2->CR1 &= ~USART_CR1_TXEIE;		//关闭TXE（发送中断）中断
		}
	}
	 
}

//与APM交换数据的口
UINT_8 DataFromAPM[APMLEN]={0};
ARMAPI void USART3_IRQHandler(void)
{

	UINT_8 temp = 0;
	USART_ClearFlag(USART3, USART_IT_IDLE); 
	if(USART_GetITStatus(USART3, USART_IT_IDLE) != RESET)  
    {  

				//读SR,DR寄存器来清除空闲中断
        temp = USART3->SR;  
        temp = USART3->DR; 
			
        DMA_Cmd(DMA1_Channel3,DISABLE);  
			  
        temp = APMLEN - DMA_GetCurrDataCounter(DMA1_Channel3);  
							
//        for (int i = 0;i < temp;i++)  
//        { 			
//						DataFromAPM[i] = GET_APM[i];  
//        }
					AnalysePacket_APM(GET_APM,temp);
//				//清空缓冲区
//				for(int i=0;i<APMLEN;i++)
//				{
//					GET_APM[i]=0;
////					DataFromAPM[i]=0;
//				}
				
				DMA1_Channel3->CNDTR =APMLEN;         
        DMA_Cmd(DMA1_Channel3,ENABLE);  
    } 

}
//光流串口中断函数
ARMAPI void UART4_IRQHandler(void)
{
		u8 com_data;
	
	if(UART4->SR & USART_SR_ORE)//ORE中断
	{
		com_data = UART4->DR;
	}

  //接收中断
	if(USART_GetITStatus(UART4,USART_IT_RXNE) )
	{
		USART_ClearFlag(UART4, USART_FLAG_TC);

		com_data = UART4->DR;
		Get_Flow_Prepare(com_data);
	}
	//发送（进入移位）中断
	if( USART_GetITStatus(UART4,USART_IT_TXE ) )
	{
//		UART4->DR = TxBuffer[TxCounter++]; //写DR清除中断标志          
//		if(TxCounter == _tcount)
//		{
//			TxCounter = _tcount=0;
//			USART2->CR1 &= ~USART_CR1_TXEIE;		//关闭TXE（发送中断）中断
//		}
	}
	 
}	
ARMAPI void UART5_IRQHandler(void)
{
	

//	if(USART_GetITStatus(UART5, USART_IT_RXNE) != RESET)
//	{ 	
//				USART_ClearFlag(UART5, USART_FLAG_TC);
//	    //ch = USART1->DR;
////			ch = USART_ReceiveData(UART5);
//	  	//printf( "%c", ch );    //将接受到的数据直接返回打印
//	} 
	 
}


//TIM6计时定时器中断
double t =0 ;
UINT_32 count=0;
ARMAPI void TIM6_IRQHandler(void)
{
	TIM_ClearFlag(TIM6, TIM_FLAG_Update);
	
	t = Sys_Time.Get_Cycle_T(0);
	count++;
	Sys_Time.Loop_check();
}

//TIM4脉冲捕获中断函数
ARMAPI void TIM4_IRQHandler(void)
{
	
	PWMIN.PWMIN_Intterupt();
}
/**
  * @brief  This function handles PPP interrupt request.
  * @param  None
  * @retval None
  */
/*void PPP_IRQHandler(void)
{
}*/

/**
  * @}
  */ 


/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
