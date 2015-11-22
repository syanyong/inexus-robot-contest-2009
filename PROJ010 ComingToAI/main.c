/* Includes ------------------------------------------------------------------*/
#include "stm32f10x.h"
#include "main.h"
#include "_mc_config.h"
#include "_glcd_config.h"
#include "_dsp_config.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define 		ADC1_DR_Address    ((uint32_t)0x4001244C)
#define 		ADC1_NbOfChannel	4
__IO uint16_t						ADC1_BUFFER[ADC1_NbOfChannel];


/*      Drive Motor Control    */
#define 	PWM_MotorLeftPin 	GPIO_Pin_6
#define 	PWM_MotorRightPin 	GPIO_Pin_7
#define 	MotorRightPin 		GPIO_Pin_5
#define 	MotorLeftPin 		GPIO_Pin_4
#define 	MotorPort 			GPIOA
#define		MaxSpeed			1000

/********* Panel I/O Interface *********/
#define 	BUTTON1				!GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_0)
#define 	BUTTON2				!GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_8)
#define 	BUTTON3				!GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_15)
#define 	BUTTON4			   	!GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_14)
#define 	LED4_On				GPIO_WriteBit(GPIOC, GPIO_Pin_8, Bit_RESET)
#define 	LED4_Off			GPIO_WriteBit(GPIOC, GPIO_Pin_8, Bit_SET)
#define 	LED3_On				GPIO_WriteBit(GPIOC, GPIO_Pin_9, Bit_RESET)
#define 	LED3_Off			GPIO_WriteBit(GPIOC, GPIO_Pin_9, Bit_SET)
#define 	LED2_On				GPIO_WriteBit(GPIOC, GPIO_Pin_10, Bit_RESET)
#define 	LED2_Off			GPIO_WriteBit(GPIOC, GPIO_Pin_10, Bit_SET)
#define 	LED1_On				GPIO_WriteBit(GPIOC, GPIO_Pin_11, Bit_RESET)
#define 	LED1_Off			GPIO_WriteBit(GPIOC, GPIO_Pin_11, Bit_SET)

/****************** MCU Configuration *******************/
static __IO uint32_t	TimingDelay;

void RCC_Configuration(void);
void GPIO_Configuration(void);
void UART1_Configuration(void);
void DMA1_Configuration(void);
void ADC1_Configuration(void);
void Delay(__IO uint32_t nTime);

/*********************** Track **************************/

/*********************** Sensor *************************/
void 		senser_showlcd(void);
int 		read_Sensor(void); 					// Read		
int 		Read_Sensor2(void);
int8_t 		measuredSensor(void);			// Read

uint8_t		DistanceRight_Flag 			= 0;
uint8_t		DistanceCenter_Flag		    = 0;
uint8_t		DistanceLeft_Flag		    = 0;
uint8_t		Color_Flag				    = 0;

				
/**
  * @brief  Main program.
  * @param  None
  * @retval None
  */
int main(void)
{
	RCC_Configuration();
  	GPIO_Configuration();
	LED4_On; LED3_On; LED2_On; LED1_On;
  	UART1_Configuration();
  	TIM_Configuration();
	DMA1_Configuration();
  	ADC1_Configuration();
  	graphic_lcd_config();
  	printf("\n\n\riNexus Robot Contest \n\r");
	NVIC_Configuration();
	DropBox();
	LED4_Off; LED3_Off; LED2_Off; LED1_Off;

  /* Infinite loop */
  while (1)
  {
		//DropBox(); 
		//Delay(1000);
		//LoadBox();
		//Delay(1000);
		printf(" %d , %d , %d , %d ::::",ADC1_BUFFER[0],ADC1_BUFFER[1],ADC1_BUFFER[2],ADC1_BUFFER[3]);
		printf("%d ,%d , %d , %d \n\r",DistanceRight_Flag,DistanceCenter_Flag,DistanceLeft_Flag,Color_Flag);
	
		if(BUTTON1){
			LED1_On;
			Delay(500);
			TrackingLeft();
		}else{
			LED1_Off;
		}
		if(BUTTON2){
			LED2_On;
			Delay(500);
			TrackingRight();
		}else{
			LED2_Off;
		}
		if(BUTTON3){
			LED3_On;
			Delay(500);
			MoveToBox();
		}else{
			LED3_Off;
		}
		if(BUTTON4){
			LED4_On;																						    
			Delay(500);
			LED4_On;
		//	MoveToPoint(1);
		 //Set_speed(0,1000);
		}else{
			LED4_Off;
		}

		
		/*Servo1_Drive(180);
		MoveToPointV2(1);
		TrackingRight();
		MoveToPointV2(1);
		TrackingLeft();
		MoveToPointV2(1);
		TrackingLeft();
		MoveToPointV2(1);
		TrackingLeft();
		MoveToPointV2(1);
		TrackingLeft();
		MoveToPointV2(1);
		TrackingRight();
		MoveToPointV2(1);
		TrackingRight();
		MoveToPointV2(1);*/	

		
  }
}

/**
  *		@brief Sensor to LCD.
  *
  */
void senser_showlcd(void)
{
int B[5],F[5],a[3];
		B[0]=GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_11);
		B[1]=GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_12);
		B[2]=GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_6);
		B[3]=GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_7);
		B[4]=GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_14);
lcd_gotoxy(0,0);
lcd_put_char(B[0]+48);
lcd_gotoxy(1*8,0);
lcd_put_char(B[1]+48);
lcd_gotoxy(2*8,0);
lcd_put_char(B[2]+48);
lcd_gotoxy(3*8,0);
lcd_put_char(B[3]+48);
lcd_gotoxy(4*8,0);
lcd_put_char(B[4]+48);


		F[0]=GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_15);
		F[1]=GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_13);
		F[2]=GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_5);
		F[3]=GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_6);
		F[4]=GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_7);

lcd_gotoxy(0*8,1);
lcd_put_char(F[0]+48);
lcd_gotoxy(1*8,1);
lcd_put_char(F[1]+48);
lcd_gotoxy(2*8,1);
lcd_put_char(F[2]+48);
lcd_gotoxy(3*8,1);
lcd_put_char(F[3]+48);
lcd_gotoxy(4*8,1);
lcd_put_char(F[4]+48);

a[2]=(B[0]*1)+(B[1]*2)+(B[2]*4)+(B[3]*8)+(B[4]*16);
a[0]=a[2]%10;

a[1]=a[2]/10;

lcd_gotoxy(3*8,2);
lcd_put_char((a[1])+48);
lcd_gotoxy(4*8,2);
lcd_put_char((a[0])+48);
//lcd_clear_screen() ; 
}
int8_t measuredSensor(void){
uint8_t	rawValue 		= 0;
int8_t	outPut			= 0;
int8_t	setPointSensor  = 9;
	rawValue  = GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_11);
	rawValue += GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_12)*2 ;
	rawValue += GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_6) *4 ;
	rawValue += GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_7) *8 ;
	rawValue += GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_14)*16;
	if(rawValue == setPointSensor){
		// Setpoint Correct
		outPut = 0;	
	}
	else if(rawValue==31){
		// All while Case
		outPut = 100;
	}
	else if((rawValue >= (  1  )) && (rawValue <= (  17  ))){
		// Normal Case
		outPut = setPointSensor - rawValue;
	}
	else{
		outPut = 0;
	}
	return outPut;
	// ERROR to Right ->> Positive.
	// ERROR to Left  ->> Negative.
}
							  
/**
  *		@brief Sensor.
  *
  */
int Read_Sensor(void)
{
int B[5],a;		
		B[0]=GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_11);
		B[1]=GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_12);
		B[2]=GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_6);
		B[3]=GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_7);
		B[4]=GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_14);
a=(B[0]*1)+(B[1]*2)+(B[2]*4)+(B[3]*8)+(B[4]*16);

return a;

}
int Read_Sensor2(void)
{
int B[5],a;		
		B[0]=GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_5);
		B[1]=GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_6);
		B[2]=GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_7);
		B[3]=GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_15);
		B[4]=GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_13);
a=(B[0]*1)+(B[1]*2)+(B[2]*4)+(B[3]*8)+(B[4]*16);
										   //PC5,PC6,PC7			 //PB15		 //PA13
return a;

}

/*------------------------------------ The End ------------------------------------------*/


/**
  * @brief  Inserts a delay time.
  * @param  nTime: specifies the delay time length, in milliseconds.
  * @retval None
  */
void Delay(__IO uint32_t nTime)
{ 
  TimingDelay = nTime;

  while(TimingDelay != 0);
}

/**
  * @brief  Decrements the TimingDelay variable.
  * @param  None
  * @retval None
  */
void TimingDelay_Decrement(void)
{
  if (TimingDelay != 0x00)
  { 
    TimingDelay--;
  }
}

/*******************************************************************************
* Function Name  : RCC_Configuration
* Description    : Configures the different system clocks.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void RCC_Configuration(void)
{
  ErrorStatus HSEStartUpStatus;
  /* RCC system reset(for debug purpose) */
  RCC_DeInit();

  /* Enable HSE */
  RCC_HSEConfig(RCC_HSE_ON);

  /* Wait till HSE is ready */
  HSEStartUpStatus = RCC_WaitForHSEStartUp();

  if(HSEStartUpStatus == SUCCESS)
  {
    /* Enable Prefetch Buffer */
    FLASH_PrefetchBufferCmd(FLASH_PrefetchBuffer_Enable);

    /* Flash 2 wait state */
    FLASH_SetLatency(FLASH_Latency_2);
 
    /* HCLK = SYSCLK */
    RCC_HCLKConfig(RCC_SYSCLK_Div1); 
  
    /* PCLK2 = HCLK */
    RCC_PCLK2Config(RCC_HCLK_Div1); 

    /* PCLK1 = HCLK/2 */
    RCC_PCLK1Config(RCC_HCLK_Div2);
    
    /* ADC 14MHz Max => 72MHz => 12MHz*/
    RCC_ADCCLKConfig(RCC_PCLK2_Div6);

    /* PLLCLK = 8MHz * 9 = 72 MHz */
    RCC_PLLConfig(RCC_PLLSource_HSE_Div1, RCC_PLLMul_9);

    /* Enable PLL */ 
    RCC_PLLCmd(ENABLE);

    /* Wait till PLL is ready */
    while(RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET)
    {
    }

    /* Select PLL as system clock source */
    RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);

    /* Wait till PLL is used as system clock source */
    while(RCC_GetSYSCLKSource() != 0x08)
    {
    }
  }
	/* Enable RCC */	

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2 | RCC_APB1Periph_TIM6 | RCC_APB1Periph_TIM3, ENABLE);	

  	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO 
						| RCC_APB2Periph_GPIOA 
  						| RCC_APB2Periph_GPIOB  
						| RCC_APB2Periph_GPIOC
						| RCC_APB2Periph_GPIOF , ENABLE);

  /* Enable DMA1 clock */
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
  
  /* Enable ADC1 clocks */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
     
  /* Enable UART1 clocks */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);

  /* Setup SysTick Timer for 1 msec interrupts  */
  if (SysTick_Config(SystemCoreClock / 1000))
  { 
    /* Capture error */ 
    while (1);
  } 
}


void DMA1_Configuration(void)
{
  DMA_InitTypeDef DMA_InitStructure;
  
  ADC_DeInit(ADC1);
  DMA_Cmd(DMA1_Channel1, DISABLE);
  DMA_StructInit(&DMA_InitStructure);
 /* DMA1 channel1 configuration ----------------------------------------------*/
  DMA_DeInit(DMA1_Channel1);
  DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)ADC1_DR_Address;
  DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)ADC1_BUFFER;
  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
  DMA_InitStructure.DMA_BufferSize = ADC1_NbOfChannel;
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
  DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
  DMA_InitStructure.DMA_Priority = DMA_Priority_High;
  DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
  DMA_Init(DMA1_Channel1, &DMA_InitStructure);
  
  /* Enable DMA1 Channel1 */
  DMA_Cmd(DMA1_Channel1, ENABLE);
}

void ADC1_Configuration(void)
{
  ADC_InitTypeDef ADC_InitStructure;

  /*
  	PB0 : ADC12_IN8
	PC2 : ADC123_IN12
	PC1	: ADC123_IN11
	PC0 : ADC123_IN10
  */
  /* DISABLE ADC1 DMA */
  ADC_DMACmd(ADC1, DISABLE);
  ADC_Cmd(ADC1, DISABLE);
  ADC_DeInit(ADC1);
  /* ADC1 configuration ------------------------------------------------------*/
  ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;
  ADC_InitStructure.ADC_ScanConvMode = ENABLE;
  ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;
  ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
  ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
  ADC_InitStructure.ADC_NbrOfChannel = ADC1_NbOfChannel;
  ADC_Init(ADC1, &ADC_InitStructure);

  /* ADC1 regular channels configuration */  
  ADC_RegularChannelConfig(ADC1, ADC_Channel_8 , 4, ADC_SampleTime_239Cycles5);	// PB0->RIGHT 
  ADC_RegularChannelConfig(ADC1, ADC_Channel_12, 3, ADC_SampleTime_239Cycles5);	// PC2->CENTER
  ADC_RegularChannelConfig(ADC1, ADC_Channel_11, 2, ADC_SampleTime_239Cycles5);	// PC1->LEFT  
  ADC_RegularChannelConfig(ADC1, ADC_Channel_10, 1, ADC_SampleTime_239Cycles5);	// PC0->COLOR

  /* Enable ADC1 DMA */
  ADC_DMACmd(ADC1, ENABLE);
  
  /* Finish Conversion */
  ADC_ITConfig(ADC1, ADC_IT_EOC, ENABLE);

  /* Enable ADC1 */
  ADC_Cmd(ADC1, ENABLE);
  
  /* Enable ADC1 reset calibaration register */   
  ADC_ResetCalibration(ADC1);
  /* Check the end of ADC1 reset calibration register */
  while(ADC_GetResetCalibrationStatus(ADC1));

  /* Start ADC1 calibaration */
  ADC_StartCalibration(ADC1);
  /* Check the end of ADC1 calibration */
  while(ADC_GetCalibrationStatus(ADC1));
     
  /* Start ADC1 Software Conversion */ 
  ADC_SoftwareStartConvCmd(ADC1, ENABLE);
}

/*******************************************************************************
* Function Name  : GPIO_Configuration
* Description    : Configures the different GPIO ports.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void GPIO_Configuration(void)
{
  	GPIO_InitTypeDef GPIO_InitStructure;
 	EXTI_InitTypeDef EXTI_InitStructure;

  	/* Configure USART1 Rx (PA.10) as input floating */
  	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
  	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  	GPIO_Init(GPIOA, &GPIO_InitStructure);
  
  	/* Configure USART1 Tx (PA.09) as alternate function push-pull */
  	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
  	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  	GPIO_Init(GPIOA, &GPIO_InitStructure);
  
  	/* Motor PWM */
  	// PA6 -> PWM  LEFT		
  	// PA7 -> PWM  RIGHT
  	/* Motor PWM */
  	GPIO_InitStructure.GPIO_Pin = PWM_MotorLeftPin|PWM_MotorRightPin;
  	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  	GPIO_Init(MotorPort, &GPIO_InitStructure);
  	/*Motor Left/Right*/
  	GPIO_InitStructure.GPIO_Pin = MotorLeftPin|MotorRightPin;
  	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  	GPIO_Init(MotorPort, &GPIO_InitStructure);

	// PA4 -> GPIO LEFT
	// PA5 -> GPIO RIGHT
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4	
								| GPIO_Pin_5;		
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;			   		
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;					
	GPIO_Init(GPIOA, &GPIO_InitStructure);	

	// PB0 -> INT0 LEFT
	// PB1 -> INT1 LEFT
	// PC4 -> INT0 RIGHT
	// PD2 -> INT1 RIGHT
	/*		GPIO Initial	   */
   	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_1; 						//PB1
  	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING; 
  	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  	GPIO_Init(GPIOB, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;					    //PC3
	GPIO_Init(GPIOC, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;					    //PC4
	GPIO_Init(GPIOC, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;						//PD2
	GPIO_Init(GPIOD, &GPIO_InitStructure);

	/*		GPIO to EXTI Initial	   */
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOB,GPIO_PinSource1);		//PB1
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOC,GPIO_PinSource3);	    //PC3
 	GPIO_EXTILineConfig(GPIO_PortSourceGPIOC,GPIO_PinSource4);	    //PC4
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOD,GPIO_PinSource2);	    //PD2
   	
	/*		EXTI Initial	   */
	EXTI_InitStructure.EXTI_Line = EXTI_Line3 		   				// EXTI_Line3
								| EXTI_Line1 						// EXTI_Line1
								| EXTI_Line2 						// EXTI_Line2
								| EXTI_Line4;						// EXTI_Line4
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStructure);

	/*Sensor init terminal left*/
	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_11|GPIO_Pin_12;				 //PA11,PA12
  	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU; 
  	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_6|GPIO_Pin_7|GPIO_Pin_14;				 //PB6,PB7,PB14
  	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU; 
  	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  	GPIO_Init(GPIOB, &GPIO_InitStructure);		   
	/*Sensor init terminal Right*/
	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_5|GPIO_Pin_6|GPIO_Pin_7;				 //PC5,PC6,PC7
  	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU; 
  	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  	GPIO_Init(GPIOC, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_15;				 //PB15
  	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU; 
  	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_13;				 //PA13
  	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU; 
  	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	/* Disable JTAG  */
	GPIO_PinRemapConfig(GPIO_Remap_SWJ_Disable, ENABLE);

	/* Button Configuration  **********************************************************/
 	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_8 | GPIO_Pin_14 | GPIO_Pin_15;
 	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
 	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
 	GPIO_Init(GPIOA, &GPIO_InitStructure);	 

	/* Configure LED --------------------------------------------------*/
 	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10 | GPIO_Pin_11;
 	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
 	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
 	GPIO_Init(GPIOC, &GPIO_InitStructure);
	GPIO_WriteBit(GPIOC, GPIO_Pin_8, Bit_SET);
	GPIO_WriteBit(GPIOC, GPIO_Pin_9, Bit_SET);
	GPIO_WriteBit(GPIOC, GPIO_Pin_10, Bit_SET);
	GPIO_WriteBit(GPIOC, GPIO_Pin_11, Bit_SET);
	 
	/*Servo*/
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_3;
  	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
 	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
 	GPIO_Init(GPIOA, &GPIO_InitStructure);

   	/* Configure ADC  */
  	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 ;
  	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
  	GPIO_Init(GPIOB, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_1 | GPIO_Pin_0;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
	GPIO_Init(GPIOC, &GPIO_InitStructure);

}

void UART1_Configuration(void)
{
  USART_InitTypeDef USART_InitStructure;
  /* USART1 configuration ----------------------------------------------------*/
  /* USART configured as follow:
        - BaudRate = 115200 baud  
        - Word Length = 8 Bits
        - One Stop Bit
        - No parity
        - Hardware flow control disabled (RTS and CTS signals)
        - Receive and transmit enabled
  */
  USART_InitStructure.USART_BaudRate = 9600;
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
  USART_InitStructure.USART_StopBits = USART_StopBits_1;
  USART_InitStructure.USART_Parity = USART_Parity_No;
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

  /* Configure USART1 */
  USART_Init(USART1, &USART_InitStructure);

  /* Enable the USART1 */
  USART_Cmd(USART1, ENABLE); 
}
/* Eng Programs ================================================================*/
#ifdef  USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *   where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}
#endif

/**
  * @}
  */


/******************* (C) COPYRIGHT 2010 STMicroelectronics *****END OF FILE****/
