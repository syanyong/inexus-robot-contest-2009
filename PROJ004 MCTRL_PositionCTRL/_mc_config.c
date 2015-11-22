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
#include "stm32f10x_tim.h"

/* Private  Define -----------------------------------------------------------------------*/
/* Private  Export Variables -------------------------------------------------------------*/
/* Private  Exports Function -------------------------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private Function ----------------------------------------------------------*/

void TIM_Configuration(void){

	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;

  	TIM_TimeBaseStructure.TIM_Period = 20000;
  	TIM_TimeBaseStructure.TIM_Prescaler = 20;
  	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);	

  	/* Time base configuration 28799*/
  	TIM_TimeBaseStructure.TIM_Period = 28799;
  	TIM_TimeBaseStructure.TIM_Prescaler = 5;
  	TIM_TimeBaseStructure.TIM_ClockDivision = 2;
  	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  	TIM_TimeBaseInit(TIM6, &TIM_TimeBaseStructure);

	TIM_ARRPreloadConfig(TIM6, ENABLE);

	TIM_ITConfig(TIM6, TIM_IT_Update, ENABLE);

	/* TIM3 enable */
  	TIM_Cmd(TIM6, ENABLE);





}

void NVIC_Configuration(void){
	NVIC_InitTypeDef NVIC_InitStructure;

	/* Configure the NVIC Preemption Priority Bits */
  	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);	

	/* Enable and set Button EXTI0_IRQn Interrupt */
	NVIC_InitStructure.NVIC_IRQChannel = EXTI3_IRQn;							//STM32 Devices IRQ Channels(refer to stm32f10x.h)
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x00;		//NVIC_Priority_Table(refer to misc.h)
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x00;			//NVIC_Priority_Table(refer to misc.h)
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;					//Enable NVIC_IRQChannel
    NVIC_Init(&NVIC_InitStructure);

	/* Enable and set Button EXTI1_IRQn Interrupt */
	NVIC_InitStructure.NVIC_IRQChannel = EXTI1_IRQn;							//STM32 Devices IRQ Channels(refer to stm32f10x.h)
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x00;	//NVIC_Priority_Table(refer to misc.h)
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x00;			//NVIC_Priority_Table(refer to misc.h)
    NVIC_InitStructure.NVIC_IRQChannelCmd = DISABLE;					//Enable NVIC_IRQChannel
    NVIC_Init(&NVIC_InitStructure);

	//LEFT
	/* Enable and set Button EXTI2_IRQn Interrupt */
	NVIC_InitStructure.NVIC_IRQChannel = EXTI2_IRQn;							//STM32 Devices IRQ Channels(refer to stm32f10x.h)
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x04;	//NVIC_Priority_Table(refer to misc.h)
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x04;			//NVIC_Priority_Table(refer to misc.h)
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;					//Enable NVIC_IRQChannel
    NVIC_Init(&NVIC_InitStructure);

	/* Enable and set Button EXTI4_IRQn Interrupt */
	NVIC_InitStructure.NVIC_IRQChannel = EXTI4_IRQn;						//STM32 Devices IRQ Channels(refer to stm32f10x.h)
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x00;	//NVIC_Priority_Table(refer to misc.h)
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x00;			//NVIC_Priority_Table(refer to misc.h)
    NVIC_InitStructure.NVIC_IRQChannelCmd = DISABLE;					//Enable NVIC_IRQChannel
    NVIC_Init(&NVIC_InitStructure);

	/* Enable and set TIM6_IRQn Interrupt */
	NVIC_InitStructure.NVIC_IRQChannel = 54;						//STM32 Devices IRQ Channels(refer to stm32f10x.h)
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x05;	//NVIC_Priority_Table(refer to misc.h)
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x06;			//NVIC_Priority_Table(refer to misc.h)
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
 
