/**
  ******************************************************************************
  * @file    		mc_config.c
  * @author  		Sarucha Yanyong.
  * @version 		V3.3.0
  * @date    		10/11/2010
  * @brief   		mc_config = microcontroller configuration.
  ******************************************************************************
  */ 
/* Include -------------------------------------------------------------------------------*/
#include "_mc_config.h"

/* Private  Define -----------------------------------------------------------------------*/
/* Private  Export Variables -------------------------------------------------------------*/
/* Private  Exports Function -------------------------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private Function ----------------------------------------------------------*/

void NVIC_Configuration(void){
	NVIC_InitTypeDef NVIC_InitStructure;

	/* Configure the NVIC Preemption Priority Bits */
  	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);	

	/* Enable and set TIM6_IRQn Interrupt */
	NVIC_InitStructure.NVIC_IRQChannel = 54;						//STM32 Devices IRQ Channels(refer to stm32f10x.h)
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x05;	//NVIC_Priority_Table(refer to misc.h)
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x06;			//NVIC_Priority_Table(refer to misc.h)
    NVIC_InitStructure.NVIC_IRQChannelCmd = DISABLE;					//Enable NVIC_IRQChannel
    NVIC_Init(&NVIC_InitStructure);

	/* ADC1 Conversion Finish*/
	NVIC_InitStructure.NVIC_IRQChannel = ADC1_2_IRQn;				//STM32 Devices IRQ Channels(refer to stm32f10x.h)
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x01;	//NVIC_Priority_Table(refer to misc.h)
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x01;			//NVIC_Priority_Table(refer to misc.h)
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;					//Enable NVIC_IRQChannel
    NVIC_Init(&NVIC_InitStructure);
}
/////////////////////////////////////////////////////////////////////////////////
PUTCHAR_PROTOTYPE
{
  /* Place your implementation of fputc here */
  /* e.g. write a character to the USART */
  USART_SendData(USART1, (uint8_t) ch);

  /* Loop until the end of transmission */
  while (USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET)
  {}

  return ch;
}

/* The End *********************************************************************/
 
