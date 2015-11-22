/**
  ******************************************************************************
  * @file    Project/STM32F10x_StdPeriph_Template/stm32f10x_it.c 
  * @author  MCD Application Team
  * @version V3.3.0
  * @date    04/16/2010
  * @brief   Main Interrupt Service Routines.
  *          This file provides template for all exceptions handler and 
  *          peripherals interrupt service routine.
  ******************************************************************************
  * @copy
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2010 STMicroelectronics</center></h2>
  */ 

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x_it.h"
#include "main.h"
#include "_mc_config.h"
#include "_dsp_config.h"


/** @addtogroup STM32F10x_StdPeriph_Template
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
GPIO_InitTypeDef GPIO_InitStructure;
EXTI_InitTypeDef EXTI_InitStructure;

/* Private define ------------------------------------------------------------*/
#define 	measurespeed
#define 	enRightInt0			GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_3)
#define 	enRightInt1			GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_1)
#define 	enLeftInt1			GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_4)
#define 	enLeftInt0			GPIO_ReadInputDataBit(GPIOD,GPIO_Pin_2)

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
uint8_t				VeloDetectPrescaler = 0;
extern 	int32_t		enCountLeft;
extern 	int32_t 	enCountRight;
int16_t				_enCountLeft;
int16_t 			_enCountRight;
extern 	int8_t		leftDirec;
extern 	int8_t		rightDirec;
int8_t				leftDirecC;
int8_t  			rightDirecC;
int	  				enVeloLeft;
int		  			enVeloRight;
extern 	int	  		SpeedLeft;
extern 	int		  	SpeedRight;
extern	int16_t		setPointRight;
extern 	int16_t		setPointLeft;

extern uint8_t pidEnable;
extern float kpR;		   		// constant for pid Right or Left Motor
extern float kiR;
extern float kdR; 
extern float kpL;
extern float kiL; 
extern float kdL;	
extern float erR;
extern float erpR;
extern float erL;
extern float erpL;	
extern int16_t pidR;
extern int16_t pidL;			
float	IntTerm_C_R = 0;
float	IntTerm_C_L = 0;

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/******************************************************************************/
/*            Cortex-M3 Processor Exceptions Handlers                         */
/******************************************************************************/

/**
  * @brief   This function handles NMI exception.
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
uint8_t iSysTick;
void SysTick_Handler(void)
{
  		TimingDelay_Decrement();
		
		if(iSysTick++ > 5){
			/* 		Start Encoder		*/
			enVeloRight = 	_enCountRight*(rightDirec);
   			enVeloLeft 	= 	_enCountLeft*(leftDirec*(-1));
							
			_enCountLeft = 0;
			_enCountRight = 0;
			SpeedRight	=	enVeloRight;
			SpeedLeft	=	enVeloLeft;
			/* 		The End			*/

			iSysTick=0;
		}
		
			if(leftDirecC >= 20){
				leftDirec = 1;
				leftDirecC = 0;
			}
			else if(leftDirecC <= -20){
				leftDirec = -1;	
				leftDirecC = 0;
			}
			//
			if(rightDirecC >= 20){
				rightDirec = 1;
				rightDirecC = 0;
			}
			else if(rightDirecC <= -20){
				rightDirec = -1;	
				rightDirecC = 0;
			} 
}

/******************************************************************************/
/*                 STM32F10x Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32f10x_xx.s).                                            */
/******************************************************************************/
/**
  *	@brief PC3 -> INT0 RIGHT
  */
void EXTI3_IRQHandler(void)
{
	if(EXTI_GetITStatus(EXTI_Line3) == SET)
	{
	EXTI_ClearFlag(EXTI_Line3);
	EXTI_ClearITPendingBit(EXTI_Line3);
	//////////////////////////////////////////////////////////////
		//printf("PC3 -> INT0 LEFT\n\r");
			_enCountRight++;
			//enCountRight++;
			// Direc
			if(!enRightInt1){
				rightDirecC++;
				enCountRight++;
			}
			else{
				rightDirecC--;
				enCountRight--;
			}
		
	}
}
/**
  *	@brief PD2 -> INT0 LEFT
  */
void EXTI2_IRQHandler(void)
{
	if(EXTI_GetITStatus(EXTI_Line2) == SET)
	{
	EXTI_ClearFlag(EXTI_Line2);
	EXTI_ClearITPendingBit(EXTI_Line2);
	///////////////////////////////////////////////////////////////
			_enCountLeft++;
			//
			if(!enLeftInt1){
				leftDirecC++;
				enCountLeft--;
			}
			else{
				leftDirecC--;
				enCountLeft++;
			}
	}
}

/**
  * @brief  This function handles TIM2_IRQ Handler.
  * @param  None
  * @retval None
  */
void TIM2_IRQHandler(void)
{

}

/**
  * @brief  This function handles TIM6_IRQ Handler.
  * @param  None
  * @retval None
  */
void TIM6_IRQHandler(void)
{
  	if (TIM_GetITStatus(TIM6, TIM_IT_Update) == SET)
  	{
    	TIM_ClearITPendingBit(TIM6, TIM_IT_Update);
		/////////////////////////////////////////// 
		/*		PID Controller	*/
		if(pidEnable){

			erL = (float)setPointLeft - enVeloLeft;
  			IntTerm_C_L += kiL*erL;
  			pidL = (kpL * erL) + IntTerm_C_L + (kdL * (erL - erpL));
  			erpL = erL;
		 		
			erR = (float)setPointRight - enVeloRight;
  			IntTerm_C_R += kiR*erR;
  			pidR = (kpR * erR) + IntTerm_C_R + (kdR * (erR - erpR));
  			erpR = erR;						   
						
			if(pidL > 1000)
				pidL = 1000;
			else if(pidL < -1000)
				pidL = -1000;

			
			if(pidR > 1000)
				pidR = 1000;
			else if(pidR < -1000)
				pidR = -1000;

			Set_speed(pidL,pidR);
			 //_eRight(pidR); _eLeft(pidL);

		}		
	}
}




/******************************************************************************/
/*                 STM32F10x Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32f10x_xx.s).                                            */
/******************************************************************************/

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


/******************* (C) COPYRIGHT 2010 STMicroelectronics *****END OF FILE****/
