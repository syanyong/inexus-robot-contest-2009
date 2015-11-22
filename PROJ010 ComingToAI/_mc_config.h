/**
  ******************************************************************************
  * @file    		mc_config.h 
  * @author  		Sarucha Yanyong.
  * @version 		V3.3.0
  * @date    		10/11/2010
  * @brief   		mc_config = microcontroller configuration.
  ******************************************************************************
  */ 
/* Include -------------------------------------------------------------------------------*/
#include "stm32f10x.h"
#include <stdio.h>

/* Private  Define -----------------------------------------------------------------------*/
/* Private  Export Variables -------------------------------------------------------------*/
/* Private  Exports Function -------------------------------------------------------------*/
void NVIC_Configuration(void);

//void delay_ms(__IO uint32_t nTime);
#ifdef __GNUC__
/* With GCC/RAISONANCE, small printf (option LD Linker->Libraries->Small printf
   set to 'Yes') calls __io_putchar() */
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */
////////////////////////////////////////////////////////////////////////////////////////////

/* The End *********************************************************************/
 
